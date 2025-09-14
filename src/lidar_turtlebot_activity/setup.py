from setuptools import find_packages, setup

package_name = 'lidar_turtlebot_activity'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vboxuser',
    maintainer_email='windks@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'lidar_plotter = lidar_turtlebot_activity.lidar_plotter:main',
        	'turtlebot_avoidance = lidar_turtlebot_activity.turtlebot_avoidance:main'
        ],
    },
)
