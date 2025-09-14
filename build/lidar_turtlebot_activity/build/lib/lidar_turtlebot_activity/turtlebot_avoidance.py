import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from sensor_msgs.msg import LaserScan

class TurtlebotAvoidance(Node):
	def __init__(self):
		super().__init__('turtlebot_avoidance')
		self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
		self.subscription = self.create_subscription(
			LaserScan,
			'/scan',
			self.lidar_callback,
			10
		)
		
		self.linear_speed = 0.2 # define  uma velocidade linear fixa em m/s
		self.andular_speed = 0.5 # define uma velocidade angular para curvas em rad/s
		self.safe_distance = 0.5 # distancia para detectar obstaculos em metros
		
	def lidar_callback(self, msg):
		# Dados LiDAR sao arrays de distance. Vou levar em consideracao apenas a frente.
		# Primeiro dando check do range dos valores frontais, ex: de -30 a 30 graus
		# O numero de pontos de dados varia, entao e preciso calcular o index
		
		# range de angulo: -30 a +30 graus em radianos
		front_angle_min = math.radians(-30)
		front_angle_max = math.radians(30)
		
		# calcular os indices correspondentes a estes angulos
		index_min = int((front_angle_min - msg.angle_min) / msg.angle_increment)
		index_max = int((front_angle_max - msg.angle_min) / msg.angle_increment)
		
		# dividir data para ter apenas os as leituras frontais
		front_distances = msg.ranges[index_min:index_max]
		
		# check para verificar se objetos estao muito proximos
		obstacle_in_front = False
		for dist in front_distances:
			# check para validar distancie e se esta entre a zona segura
			if not math.isinf(dist) and not math.isnan(dist) and dist < self.safe_distance:
				obstacle_in_front = True
				break
				
		# Cria um Twist message para controlar o robo
		twist_msg = Twist()
		
		if obstacle_in_front:
			self.get_logger().info('Obstacle detected! Truning...')
			twist_msg.linear.x = 0.0 # para o movimento frontal
			twsit_msg.angular.z = self.angular_speed # comeca a virar
		else:
			self.get_logger().info('Clear path ahead, moving forward!')
			twist_msg.linear.x = self.linear_speed # move para frente
			twist_msg.angular.z = 0.0 # para de virar
			
		# publica o comando
		self.publisher.publish(twist_msg)
		
def main(args=None):
	rclpy.init(args=args)
	turtlebot_avoidance_node = TurtlebotAvoidance()
	rclpy.spin(turtlebot_avoidance_node)
	turtlebot_avoidance_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()
		
