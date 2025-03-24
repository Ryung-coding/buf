from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Optitrack Node
        Node(
            package='mocap',
            executable='mocap_worker',
            name='optitrack_node'
        ),

        # IMU Node
        Node(
            package='imu_worker',
            executable='imu_worker',
            name='imu_node'
        ),

        # SBUS Worker Node
        Node(
            package='sbus',
            executable='sbus_worker',
            name='sbus_node',
        ),

        # ARM Changer Node
        Node(
            package='arm_changer',
            executable='arm_changer',
            name='arm_changing_node',
        ),

        # Controller Node
        Node(
            package='controller',
            executable='controller_worker',
            name='controller_node'
        ),

        # Allocator Node
        Node(
            package='allocator',
            executable='allocator_worker',
            name='allocator_node'
        ),

        # Dynamixel Node
        Node(
            package='dynamixel_worker',
            executable='dynamixel_worker',
            name='dynamixel_node'
        ),

        # Teensy Node
        Node(
            package='teensy_sender',
            executable='teensy_worker',
            name='teensy_node'
        ),

        # Watchdog Node
        Node(
            package='watchdog_manager',
            executable='watchdog_worker',
            name='watchdog_node'
        ),
        
        # MuJoCo Node
        Node(
            package='mujoco_sim',
            executable='mujoco_node',
            name='mujoco_node',
            output = 'screen'
        ),

        # PID Debugger Node
        Node(
            package='pid_debugger',
            executable='pid_debugger',
            name='pid_debugger_node',
            output='screen'
        ),

    ])