import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class HybridController(Node):
	def __init__(self):
		super().__init__("hybrid_controller")
		
		# Publicador para o topico de velocidade de comando
		self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
		
		# Assinante para o topico de odemetria (posicao do robo)
		self.odom_sub = self.create_subscription(
			Odometry,
			'odom',
			self.odom_callback,
			10
		)
		
		# Assinante para o topico do sensor LiDAR (scan)
		self.scan_sub = self.create_subscription(
			LaserScan,
			'scan',
			self.scan_callback,
			10
		)
		
		# variaveis de estado
		self.current_x = 0.0
		self.current_y = 0.0
		self.current_theta = 0.0
		self.min_distance = float('inf')
		
		# Parametros PID
		self.kp_linear = 0.5 # ganho proporcional para velocidade linear
		self.kp_angular = 1.0 # ganho proporcional para velocidade angular
		self.ki_angular = 0.05
		self.kd_angular = 0.1
		self.last_error_angular = 0.0
		self.integral_error_angular = 0.0
		
		# Ponto de destino
		self.goal_x = 1.0
		self.goal_y = 1.0
		self.goal_tolerance = 0.1
		
		# Parametros de desvio de obstaculo ("Fuzzy" simplificado)
		self.obstacle_threshold = 0.5 # Distancia em metros para comecar a desviar
		self.fuzzy_kp = 0.8 # ganho de controle para desvio
		
		# timer para o loop de controle
		self.timer = self.create_timer(0.1, self.control_loop)
		
		self.get_logger().info("Hybrid Controller Node started")
		
		
	def odom_callback(self, msg):
		self.current_x = msg.pose.pose.position.x
		self.current_y = msg.pose.pose.position.y
		orientation_q = msg.pose.pose.orientation
		siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
		cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
		self.current_theta = math.atan2(siny_cosp, cosy_cosp)
		
	def scan_callback(self, msg):
		# apenas considero os 90 graus da frente (45 graus para cada lado)
		# o turtlebot3 tem 360 pontos, 0 e a frente
		min_front_dist = min(msg.ranges[0:45] + msg.ranges[315:360])
		self.min_distance = min_front_dist if min_front_dist > msg.range_min and min_front_dist < msg.range_max else float('inf')
		
	def pid_control(self):
		# logica PID para navegacao ate o ponto
		dx = self.goal_x - self.current_x
		dy = self.goal_y - self.current_y
		distance_to_goal = math.sqrt(dx**2 + dy**2)
		
		# se chegou ao destino, para o robo
		if distance_to_goal < self.goal_tolerance:
			self.stop_robot()
			return
			
		angle_to_goal = math.atan2(dy, dx)
		error_angular = angle_to_goal - self.current_theta
		
		# Normalizar o angulo para o intervalo [-pi, pi]
		if error_angular > math.pi:
			error_angular -= 2 * math.pi
		elif error_angular < -math.pi:
			error_angular += 2 * math.pi
			
		# Calcular os termos PID para o controle angular
		self.integral_error_angular += error_angular
		derivative_error_angular = error_angular - self.last_error_angular
		self.last_error_angular = error_angular
		
		# logica de controle: a valocidade linear e proporcional a distancia, e a velocidade angular e controla pelo PID
		linear_vel = self.kp_linear * distance_to_goal
		angular_vel = (self.kp_angular * error_angular) + (self.ki_angular * self.integral_error_angular) + (self.kd_angular * derivative_error_angular)
		
		# Limita as velocidades para evitar movimentos muito bruscos
		twist = Twist()
		twist.linear.x = min(linear_vel, 0.22)
		twist.angular.z = min(angular_vel, 2.84)
		self.cmd_vel_pub.publish(twist)
		
	def fuzzy_obstacle_avoidance(self):
		# logica de desvio de obstaculo baseada na distancia frontal
		twist = Twist()
		
		if self.min_distance > self.obstacle_threshold:
			# se o obstaculo esta longe, ainda pode se mover um pouco para frente
			twist.linear.x = 0.1
		else:
			# se o obstaculo esta perto, para e gira para desviar
			twist.linear.x = 0.0
			# a velocidade angular e inversamente proporcional a distancia
			twist.angular.z = self.fuzzy_kp / self.min_distance
			
		self.cmd_vel_pub.publish(twist)
		
	def stop_robot(self):
		twist = Twist()
		twist.linear.x = 0.0
		twist.angular.z = 0.0
		self.cmd_vel_pub.publish(twist)
		self.get_logger().info("Target position rechead")
		
	def control_loop(self):
		# a logica de combinacao de mundos
		if self.min_distance < self.obstacle_threshold:
			self.get_logger().info("Obstacle detected: Switching to Fuzzy (avoidance) control")
			self.fuzzy_obstacle_avoidance()
		else:
			self.get_logger().info("No obstacles. Switching to PID (position) control")
			self.pid_control()
			
def main(args=None):
	rclpy.init(args=args)
	hybrid_controller = HybridController()
	rclpy.spin(hybrid_controller)
	hybrid_controller.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
