import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquarePublisher(Node):
	def __init__(self):
		super().__init__('square_publisher')
		self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
		self.timer = self.create_timer(1.0, self.move_square) # Time para mover a tartaruga
		# sequencia de comandos: [linear_x, angular_z]
		self.commands = [
			(2.0,0.0), # andar para frente
			(0.0, 1.57), # Girar 90 graus
			(2.0,0.0),
			(0.0,1.57),
			(2.0,0.0),
			(0.0,1.57),
			(2.0,0.0),
			(0.0,1.57),
			(0.0,0.0) # parar
		]
		self.step_index = 0
	
		
	def move_square(self):
		if self.step_index < len(self.commands):
			linear_x, angular_z = self.commands[self.step_index]
			
			msg = Twist()
			msg.linear.x = linear_x
			msg.angular.z = angular_z
			self.publisher_.publish(msg)
			
			self.get_logger().info(f'Passo{self.step_index}: Publicando comando linear.x={linear_x:.2f}, angular.z={angular_z:.2f}')
			
			self.step_index += 1
		else:
			self.get_logger().info('Quadrado completo. Parando o no')
			# Para encerrar o no
			self.timer.cancel()
			self.destroy_node()
			
			
	def main(args=None):
		rclpy.init(args=args)
		node = SquarePublisher()
		try:
			rclpy.spin(node)
		except SystemExit:
			rclpy.get_logger('square_publisher').info('No encerrado')
		finally:
			rclpy.shutdown()
		
	if __name__ == '__main__':
		main()
