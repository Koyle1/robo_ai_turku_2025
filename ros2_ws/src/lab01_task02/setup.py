from setuptools import find_packages, setup

package_name = 'lab01_task02'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy', 'custom_interface'],
    zip_safe=True,
    maintainer='felix',
    maintainer_email='coyfelix7@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = lab01_task02.publisher:main',
            'subscriber = lab01_task02.subscriber:main',
        ],
    },
)
