import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt

class LidarPlotter(Node):
	def __init__(self):
		super().__init__('lidar_plotter')
		self.subscription = self.create_subscription(
			LaserScan,
			'/scan', # topic do sensor LiDAR
			self.lidar_callback,
			10
		)
		self.points_x = []
		self.points_y = []
		
	def lidar_callback(self,msg):
		self.get_logger().info('Recebendo dados LiDAR...')
		
		# Limpar pontos anteriores
		self.points_x = []
		self.points_y = []
		
		# Converter coordenadas polares para cartesianas
		for i, distance in enumerate(msg.ranges):
			if distance > msg.range_min and distance < msg.range_max:
				angle = msg.angle_min + i * msg.angle_increment
				x = distance * math.cos(angle)
				y = distance * math.sin(angle)
				self.points_x.append(x)
				self.points_y.append(y)
	
		# Plotar dados
		plt.figure(figsize = (8,8))
		plt.scatter(self.points_x, self.points_y, s = 1)
		plt.title('LiDAR 2D Map')
		plt.xlabel('X (m)')
		plt.ylabel('Y (m)')
		plt.grid(True)
		plt.axis('equal')
		plt.show(block=False)
		plt.pause(0.01)
		
def main(args=None):
	rclpy.init(args=args)
	lidar_plotter_node = LidarPlotter()
	rclpy.spin(lidar_plotter_node)
	lidar_plotter_node.destroy_node()
	rclpy.shutdown()
	
if __name__ == 'main':
	main()
