from setuptools import find_packages, setup

package_name = 'turtle_sim_pkg'

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
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'pose_subscriber = turtle_sim_pkg.pose_subscriber:main',
        	'draw_square = turtle_sim_pkg.draw_square:main',
        	'go_to_goal = turtle_sim_pkg.go_to_goal:main'
        ],
    },
)
