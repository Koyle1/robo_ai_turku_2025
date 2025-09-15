from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    package_name = 'lab01_task05'
    params_file = os.path.join(
        os.path.dirname(__file__),  # assumes params.yaml lives next to this launch file
        'params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_from',
            default_value='20',
            description='Number to count down from'
        ),

        DeclareLaunchArgument(
            'cancel_after',
            default_value='8',
            description='Request cancellation after X seconds'
        ),

        DeclareLaunchArgument(
            'countdown_interval',
            default_value='1.0',
            description='Time between countdown steps (seconds)'
        ),

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

        # Basic Countdown Client (no cancellation)
        Node(
            package=package_name,
            executable='countdown_action_client',
            name='basic_countdown_client',
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
