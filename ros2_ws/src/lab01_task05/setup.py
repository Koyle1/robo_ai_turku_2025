from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'lab01_task05'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'custom_interface', 'rclpy'],
    zip_safe=True,
    maintainer='felix',
    maintainer_email='coyfelix7@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'countdown_action_server = lab01_task05.countdown_action_server:main',
            'countdown_action_client = lab01_task05.countdown_action_client:main',
            'cancel_service_server = lab01_task05.cancel_service_server:main',
            'cancel_service_client = lab01_task05.cancel_service_client:main',
        ],
    },
)
