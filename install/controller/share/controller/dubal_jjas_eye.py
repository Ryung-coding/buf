from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    bag_directory = os.path.expanduser("~/Desktop/bag")

    return LaunchDescription(
        [
            Node(
                package='serial_reciver',
                executable='serial_reciver_node',
                name='serial_reciver_node'
            ),
            Node(
                package='controller',
                executable='controller_main',
                name='controller_main'
            ),
            Node(
                package='dubal_eye_AND_web',  
                executable='sensor_subscriber', 
                name='sensor_subscriber'
            ),
			#Node(
			#package='localizer',  
			#executable='odometry_node', 
			#name='odometry_node'
			#)
        ]
    )
