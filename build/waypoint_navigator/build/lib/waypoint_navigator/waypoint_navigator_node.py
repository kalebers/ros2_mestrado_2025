import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class WaypointNavigator(Node):
	def __init__(self):
		super().__init__('waypoint_navigator')
		
		self.waypoints = [
			(1.0,0.0),
			(1.0,1.0),
			(0.0,1.0),
			(0.0,0.0)
		]
		self.current_waypoint_index = 0
		self.waypoint_reached_tolerance = 0.2 # Tolerancia em metros
		
		# publisher para o topico de destino
		self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
		
		# subscriber para o topico de odometria do robo
		self.odom_subscriber = self.create_subscription(
			Odometry,
			'/odom',
			self.odom_callback,
			10
		)
		
		# timer para publicar o primeiro waypoint
		self.initial_goal_timer = self.create_timer(1.0, self.publish_initial_goal)
		self.get_logger().info('Navegador de Waypoints iniciado')
		
	def publish_initial_goal(self):
		if self.current_waypoint_index == 0:
			self.get_logger().info(f'Publicando o primeiro waypoint..')
			self.publish_next_goal()
			self.initial_goal_timer.cancel() # cancela o timer apos a ptimeira publicacao
			
	def publish_next_goal(self):
		if self.current_waypoint_index < len(self.waypoints):
			x, y = self.waypoints[self.current_waypoint_index]
			
			goal_msg = PoseStamped()
			goal_msg.header.frame_id = "map"
			goal_msg.header.stamp = self.get_clock().now().to_msg()
			goal_msg.pose.position.x = x
			goal_msg.pose.position.y = y
			goal_msg.pose.orientation.w = 1.0 # sem rotacao especifica
			
			self.goal_publisher.publish(goal_msg)
			self.get_logger().info(f'Publicado waypoint [{self.current_waypoint_index + 1}/{len(self.waypoints)}] em x={x}, y={y}')
		else:
			self.get_logger().info('Todos os waypoints foram alcancados')
			
	def odom_callback(self, msg):
		if self.current_waypoint_index < len(self.waypoints):
			# posicao atual do robo
			current_x = msg.pose.pose.position.x
			current_y = msg.pose.pose.position.y
			
			# posicao do waypoint de destino
			target_x, target_y = self.waypoints[self.current_waypoint_index]
			
			# calcular a distancia euclidiana ate o waypoint
			distance = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
			
			if distance < self.waypoint_reached_tolerance:
				self.get_logger().info(f'Waypoint [{self.current_waypoint_index + 1}] alcancado. Distancia {distance:.2f} m')
				self.current_waypoint_index += 1
				self.publish_next_goal()
				
			# se todos os waypoints estiverm alcancados, o callback continua a recerver mensgaens mas nao faz nada
			
def main(args=None):
	rclpy.init(args=args)
	waypoint_navigator = WaypointNavigator()
	rclpy.spin(waypoint_navigator)
	waypoint_navigator.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
			
			
			
