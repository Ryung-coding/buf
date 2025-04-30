from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition

def validate_mode(context, *args, **kwargs):
    # Retrieve the mode value from the launch configuration
    mode_val = LaunchConfiguration('mode').perform(context)
    # Check if the mode is either 'sim' or 'real'
    if mode_val not in ['sim', 'real']:
        # Raise an error to abort the launch
        raise RuntimeError("Invalid mode provided: {}. Please use 'sim' or 'real'.".format(mode_val))
    # If valid, simply return an empty list (no additional actions needed)
    return []

def generate_launch_description():
    # Declare a launch argument 'mode' with default value 'sim'
    mode_arg = DeclareLaunchArgument(
        'mode',
        description='Launch mode: sim or real'
    )

    mode = LaunchConfiguration('mode')

    nodes = [
        # MuJoCo Node (Run only when mode==sim)
        Node(
            package='mujoco_sim',
            executable='mujoco_node',
            name='mujoco_node',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"])),
        ),

        # Optitrack Node
        Node(
            package='mocap',
            executable='mocap_worker',
            name='optitrack_node',
            parameters=[{'mode': mode}],
        ),

        # IMU Node
        Node(
            package='imu_worker',
            executable='imu_worker',
            name='imu_node',
            parameters=[{'mode': mode}],
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
            package='controller_geom',
            executable='controller_worker_geom',
            name='controller_node',
        ),

        # Allocator Node
        Node(
            package='allocator',
            executable='allocator_worker',
            name='allocator_node',
        ),

        # Dynamixel Node
        Node(
            package='dynamixel_worker',
            executable='dynamixel_worker',
            name='dynamixel_node',
            parameters=[{'mode': mode}],
        ),

        # Teensy Node
        Node(
            package='teensy_sender',
            executable='teensy_worker',
            name='teensy_node',
            parameters=[{'mode': mode}],
        ),

        # Watchdog Node
        Node(
            package='watchdog_manager',
            executable='watchdog_worker',
            name='watchdog_node',
        ),

        # GUI Node
        Node(
            package='gui',
            executable='gui',
            name='gui_node',
        ),
    ]

    return LaunchDescription([
        mode_arg,
        OpaqueFunction(function=validate_mode),
        *nodes,
    ])