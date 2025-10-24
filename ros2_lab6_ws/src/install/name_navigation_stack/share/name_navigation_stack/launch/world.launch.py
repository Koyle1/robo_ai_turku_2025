from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    custom_pkg_dir = get_package_share_directory('name_navigation_stack')
    world_file = os.path.join(custom_pkg_dir, 'lab6_task1_world', 'lab6_task1_world.world')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(custom_pkg_dir, 'lab6_task1_world')),

        # Launch Gazebo with your world only
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file],
            output='screen'
        )
    ])
