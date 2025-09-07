from setuptools import find_packages, setup

package_name = 'task5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'custom_interface'],
    zip_safe=True,
    maintainer='michael',
    maintainer_email='m.krone0402@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = task5.countdown_server:main',
            'client = task5.countdown_client:main',
            'cancel_client = task5.countdown_cancel_client:main',
        ],
    },
)
