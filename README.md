# ROS2 repository and workspaces
	- This repository have all ROS2 workspaces used in my Robot Systems class at PUCPR in 	 	2025 for my masters mandatory classes.
	- I'm using Ubuntu 22.04 with ROS2 Humble and Gazebo.
		- This workspaces are only for study purposes.
	- To run the files:
	= TurtleSim activities class 02:
		- Activity 1:
			- ros2 run turtlesim turtlesim_node
			- ros2 run turtle_sim_pkg draw_square
			- ros2 run turtle_sim_pkg pose_subscriber
		- Activity 2:
			- ros2 run turtlesim turtlesim_node
			- ros2 run turtle_sim_pkg go_to_goal <value><value>
			
	- LiDAR activity class 03:
		- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
		- ros2 run lidar_turtlebot_activity lidar_plotter
		- ros2 run lidar_turtlebot_activity turtlebot_avoidance
		
	- Fuzzy and turtleBot3 activitz class 05:
		export TURTLEBOT3_MODEL=waffle_pi
		ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
		
		ros2 run turtle_control hybrid_control
		
		ros2 topic echo /cmd_vel #to check the robot odometry
