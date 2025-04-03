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
        # Optitrack Node
        Node(
            package='mocap',
            executable='mocap_worker',
            name='optitrack_node',
            parameters=[{'mode': mode}],
            # prefix='chrt -f 90',
        ),

        # IMU Node
        Node(
            package='imu_worker',
            executable='imu_worker',
            name='imu_node',
            parameters=[{'mode': mode}],
            # prefix='chrt -f 90',
        ),

        # SBUS Worker Node
        Node(
            package='sbus',
            executable='sbus_worker',
            name='sbus_node',
            # prefix='chrt -f 88',
        ),

        # ARM Changer Node
        Node(
            package='arm_changer',
            executable='arm_changer',
            name='arm_changing_node',
            # prefix='chrt -f 88',
        ),

        # Controller Node
        Node(
            package='controller_pid',
            executable='controller_worker_pid',
            name='controller_node',
            # prefix='chrt -f 92',
        ),

        # Allocator Node
        Node(
            package='allocator',
            executable='allocator_worker',
            name='allocator_node',
            # prefix='chrt -f 91',
        ),

        # Dynamixel Node
        Node(
            package='dynamixel_worker',
            executable='dynamixel_worker',
            name='dynamixel_node',
            parameters=[{'mode': mode}],
            # prefix='chrt -f 91',
        ),

        # Teensy Node
        Node(
            package='teensy_sender',
            executable='teensy_worker',
            name='teensy_node',
            parameters=[{'mode': mode}],
            # prefix='chrt -f 91',
        ),

        # Watchdog Node
        Node(
            package='watchdog_manager',
            executable='watchdog_worker',
            name='watchdog_node',
            # prefix='chrt -f 88',
        ),
        
        # MuJoCo Node (Run only when mode==sim)
        Node(
            package='mujoco_sim',
            executable='mujoco_node',
            name='mujoco_node',
            output='screen',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"])),
            # prefix='chrt -f 89',
        ),

        # PID Debugger Node
        Node(
            package='pid_debugger',
            executable='pid_debugger',
            name='pid_debugger_node',
            output='screen',
            # prefix='chrt -f 87',
        ),
    ]

    return LaunchDescription([
        mode_arg,
        OpaqueFunction(function=validate_mode),
        *nodes,
    ])