from setuptools import setup
import os
from glob import glob

package_name = 'name_navigation_stack'

# Collecting the resource, world, sdf, config, and launch files
world_files = glob('src/name_navigation_stack/lab6_task1_world/*.world')
sdf_files = glob('src/name_navigation_stack/lab6_task1_world/*.sdf')
config_files = glob('src/name_navigation_stack/lab6_task1_world/*.config')
launch_files = glob('src/name_navigation_stack/launch/*.launch.py')
resource_files = glob('src/name_navigation_stack/resource/*')

# Worker nodes (assuming you have Python ROS nodes)
python_nodes = glob('src/name_navigation_stack/*.py')

# Define your package's setup
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Include the ROS package
    py_modules=[os.path.basename(node)[:-3] for node in python_nodes],  # Worker nodes
    data_files=[
        # Copy world and related files to the install directory
        ('share/' + package_name + '/lab6_task1_world', world_files + sdf_files + config_files),
        # Copy launch files to the install directory
        ('share/' + package_name + '/launch', launch_files),
        # Copy other resources to the install directory
        ('share/' + package_name + '/resource', resource_files),
    ],
    install_requires=['setuptools', 'rclpy', 'nav2_core'],  # Add ROS dependencies as needed
    zip_safe=True,
    # Define the entry points for your ROS2 nodes
    entry_points={
        'console_scripts': [
            'mapper = name_navigation_stack.mapper:main',  # Adjust these based on your actual nodes
            'visualize = name_navigation_stack.visualize:main',
            'wp_follower = name_navigation_stack.wp_follower:main',
        ],
    },
)
