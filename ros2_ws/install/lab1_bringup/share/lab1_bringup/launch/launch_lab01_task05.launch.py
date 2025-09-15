# system.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'lab01_task05'  

    # Compute default params file location
    try:
        pkg_share = get_package_share_directory(package_name)
        default_params_path = os.path.join(pkg_share, 'config', 'params.yaml')
        print(f"Default params path: {default_params_path}")
    except Exception:
        # fallback to launch file directory
        default_params_path = os.path.join(os.path.dirname(__file__), 'params.yaml')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Path to a params YAML file'
    )

    params_file = LaunchConfiguration('params_file')

    # optional: log a message if file is missing (helps avoid rcl warnings)
    warn_action = LogInfo(
        msg=[f'Using params file: ', params_file]
    )

    return LaunchDescription([
        declare_params_file,
        warn_action,

        # Cancel Service Server
        Node(
            package=package_name,
            executable='cancel_service_server',
            name='cancel_service_server',
            parameters=[params_file],
            output='screen'
        ),

        # Countdown Action Server
        Node(
            package=package_name,
            executable='countdown_action_server',
            name='countdown_action_server',
            parameters=[params_file],
            output='screen'
        ),

        # Cancel Service Client (sends goal and requests cancellation)
        Node(
            package=package_name,
            executable='cancel_service_client',
            name='cancel_service_client',
            parameters=[params_file],
            output='screen'
        ),
    ])
