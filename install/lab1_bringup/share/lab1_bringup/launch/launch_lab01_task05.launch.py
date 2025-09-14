from launch import LaunchDescription 
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description(): 
    #instantiate a LaunchDescription object 
    ld = LaunchDescription()
    
    # Get package directory
    package_name = 'lab01_task05'
    
    # Get parameter file path
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'countdown_params.yaml'
    )
      
    server_node = Node(
        package=package_name,     
        executable='countdown_server',   
        name='CountdownActionServer',            
        output='screen',
        parameters=[config_file]
    )
                                        
    client_node = Node(
        package=package_name,
        executable='countdown_client',  
        name='CountdownActionClient',   
        output='screen',
        parameters=[config_file]
    )
    
    cancel_client_node = Node(
        package=package_name,
        executable='countdown_cancel_client',  
        name='CountdownCancelClient',   
        output='screen',
        parameters=[config_file]
    )
    
    # add nodes to launch description 
    ld.add_action(server_node)
    ld.add_action(cancel_client_node)  # Using cancel client by default
    
    return ld 