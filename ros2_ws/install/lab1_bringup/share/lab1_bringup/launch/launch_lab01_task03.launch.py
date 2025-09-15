import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('lab01_task03')
    
    # Launch configurations
    use_terminals = LaunchConfiguration('use_terminals')
    params_file = LaunchConfiguration('params_file')
    
    # Declare launch arguments
    declare_use_terminals_cmd = DeclareLaunchArgument(
        'use_terminals',
        default_value='0',
        description='Launch each node in a separate terminal (1=yes, 0=no)'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'params.yaml'),
        description='Full path to the ROS2 parameters file to use'
    )
    
    # Service server node - Terminal execution
    service_server_terminal = ExecuteProcess(
        condition=IfCondition(use_terminals),
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'lab01_task03', 'server'],
        name='server_terminal',
        output='screen'
    )
    
    # Service server node - Normal execution
    service_server_node = Node(
        condition=UnlessCondition(use_terminals),
        package='lab01_task03',
        executable='server',
        name='DistanceServer',
        output='screen'
    )
    
    # Service client node - Terminal execution
    service_client_terminal = ExecuteProcess(
        condition=IfCondition(use_terminals),
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'lab01_task03', 'client', 
             '--ros-args', '--params-file', os.path.join(bringup_dir, 'config', 'params.yaml')],
        name='client_terminal',
        output='screen'
    )
    
    # Service client node - Normal execution
    service_client_node = Node(
        condition=UnlessCondition(use_terminals),
        package='lab01_task03',
        executable='client',
        name='DistanceClient',
        output='screen',
        parameters=[params_file]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Add the commands to the launch description
    ld.add_action(declare_use_terminals_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(service_server_terminal)
    ld.add_action(service_server_node)
    ld.add_action(service_client_terminal)
    ld.add_action(service_client_node)
    
    return ld