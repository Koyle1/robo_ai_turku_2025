import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # === Package paths ===
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_my_world = get_package_share_directory('name_navigation_stack')  # <-- your package name

    # === Launch configurations ===
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # === World file path ===
    world = os.path.join(pkg_my_world, 'lab6_task1_world', 'new_world.world')

    # === Launch Gazebo server and client ===
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # === Robot State Publisher ===
    urdf_file_name = 'turtlebot3_waffle.urdf'
    urdf = os.path.join(pkg_turtlebot3_description, 'urdf', urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
        }]
    )

    # === Spawn TurtleBot3 Waffle Pi ===
    turtlebot3_model_path = os.path.join(
        pkg_turtlebot3_gazebo, 'models', 'turtlebot3_waffle_pi', 'model.sdf'
    )

    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_waffle_pi',
            '-file', turtlebot3_model_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.1'
        ],
        output='screen'
    )

    # === Final LaunchDescription ===
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='x position'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='y position'),

        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
    ])
