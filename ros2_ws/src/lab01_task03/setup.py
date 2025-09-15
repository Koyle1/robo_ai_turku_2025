from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lab01_task03'

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
    install_requires=['setuptools', 'custom_interface', 'rclpy', 'numpy'],
    zip_safe=True,
    maintainer='felix',
    maintainer_email='coyfelix7@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = lab01_task03.client:main',
            'server = lab01_task03.server:main',
        ],
    },
)