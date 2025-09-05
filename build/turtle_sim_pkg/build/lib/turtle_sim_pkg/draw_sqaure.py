import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquarePublisher(Node):
	def __init__(self):
		super().__init__('square_publisher')
		self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
		self.timer = self.create_timer(0.5, self.move_square)
		self.step = 0
		self.count = 0
		
	def move_square(self):
		msg = Twist()
		if self.step % 2 == 0: # anda para  frente
			msg.linear.x = 2.0
			msg.angular.z = 0.0
			self.count += 1
		else: # gira 90 graus
			msg.linear.x = 0.0
			msg.angular.z = 1.57
			self.count += 1
			
		self.publisher_.publish(msg)
		
		# alterna passos: frente (x segundos)/gira (alguns segundos)
		if self.count > 4:
			self.count = 0
			self.step +=1
			
	def main(args=None):
		rclpy.init(args=args)
		node = SquarePublisher()
		rclpy.spin(node)
		node.destroy_node()
		rclpy.shutdown()
		
	if __mname__ == '__name__':
		main()
