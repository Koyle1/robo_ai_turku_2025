from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab1_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[ 
        ('share/ament_index/resource_index/packages', 
        ['resource/' + package_name]), 
        ('share/' + package_name, ['package.xml']), 
        # Include launch files 
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), 
        # Include config files 
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
    ],
     install_requires=['setuptools', 'rclpy', 'launch', 'launch_ros', 'sensor_msgs', 'std_msgs'],
    zip_safe=True,
    maintainer='felix',
    maintainer_email='coyfelix7@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
