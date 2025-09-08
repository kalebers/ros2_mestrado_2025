import rclpy
from rclpy import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys

class GoToGoal(Node):
	def __init__(self, goal_x, goal_y):
		super().__init__('go_to_goal')
		self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
		self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback,10)
		
		self.pose = Pose()
		self.goal_x = goal_x
		self.goal_y = goal_y
		
		self.get_logger().info(f'No GoToGoal iniciado. Indo para a posicao ({self.goal_x:.2f}, {self.goal_y:.2f}).')
		
	def pose_callback(self, msg):
		self.pose = msg
		self.move_to_goal()
		
	def move_to_goal(self):
		msg = Twist()
		distance = math.sqrt((self.goal_x - self.pose.x) ** 2 + (self.goal_y - self.pose.y) ** 2)
		
		if distance > 0.1: # se ainda nao chegou
			angle_to_goal = math.atan2(self.goal_y - self.pose_y, self.goal_x - self.pose.x)
			msg.linear.x = 1.5 * distance
			msg.angular.z = 4.0 * (angle_to_goal - self.pose.theta)
		else:
			msg.linear.x = 0.0
			msg.angular.z = 0.0
			
			self.get_logger().info(f'Objetivo atingido! Posicao final: ({self.pose.x:.2f}, {self.pose.y:.2f}).')
			self.destroy_node()
			return
			
		self.publisher_.publish(msg)
		
	def main(args=None):
		rclpy.init(args=args)
		#verifica se os argumentos foram passados
		if len(sys.argv) != 3:
			print("Uso: ros2 run <nome_do_pacote> go_to_goal <x> <y>")
			rclpy.shutdown()
			sys.exit(1)
		try:
			goal_x = float(sys.argv[1])
			goal_y = float(sys.argv[2])
		except:
			print("Erro, as coordenadas devem ser numeros")
			rclpy.shutdown()
			sys.exit(1)
			
		node = GoToGoal(goal_x, goal_y)
		rclpy.spin(node)
		rclpy.shutdown()
		
	if __name__ == '__main__':
		main()
