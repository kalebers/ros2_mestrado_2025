import rclpy
from rclpy import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class GoToGoal(Node):
	def __init__(self):
		super().__init__('go_to_goal')
		self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
		self.subscription = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback,10)
		
		self.pose = Pose()
		self.goal_x = 5.0 # posicao alvo x
		self.goal_y = 5.0 # posicao alvo y
		
	def pose_callback(self, msg):
		self.pose = msg
		self.move_to_goal()
		
	def move_to_goal(self):
		msg = Twist()
		distance = math.sqrt((self.goal_x - self.pose.x) ** 2 + (self.goal_y - self.pose.y) ** 2)
		
		if distance > 0.1: # se ainda nao chegou
			angle_to_goal = math.atan2(self.goal_y - self.pose_y, self.goal_x - self.pose.x)
			msg.linear.x = 1.5 * distance
			msg.angular.z = 4.0 * (angles_to_goal - self.pose.theta)
		else:
			msg.linear.x = 0.0
			msg.angular.z = 0.0
			
		self.publisher_.publish(msg)
		
	def main(args=None):
		rclpy.init(args=args)
		node = GoToGoal()
		rclpy.spin(node)
		node.destroy_node()
		rclpy.shutdown()
		
	if __name__ == '__name__':
		main()
