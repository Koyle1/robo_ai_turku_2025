from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'group_13_navigation_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch',
	'*.py'))),
	(os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds',
	'*.world'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bot',
    maintainer_email='bot@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'mapper = group_13_navigation_stack.mapper:main',  # Adjust these based on your actual nodes
        	'visualize = group_13_navigation_stack.trajectory_visualization:main',
        	'wp_follower = group_13_navigation_stack.waypoint_follower:main',
        ],
    },
)
