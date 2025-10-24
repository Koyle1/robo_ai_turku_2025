from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Custom package with your world
    custom_pkg_dir = get_package_share_directory('name_navigation_stack')
    gazebo_pkg_dir = get_package_share_directory('turtlebot3_gazebo')

    # Path to your world file
    world_file = os.path.join(custom_pkg_dir, 'lab6_task1_world', 'lab6_task1_world.world')

    # Path to the TurtleBot3 world launch file
    tb3_world_launch = os.path.join(gazebo_pkg_dir, 'launch', 'turtlebot3_world.launch.py')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(custom_pkg_dir, 'lab6_task1_world')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_world_launch),
            launch_arguments={'world': world_file}.items()
        )
    ])
