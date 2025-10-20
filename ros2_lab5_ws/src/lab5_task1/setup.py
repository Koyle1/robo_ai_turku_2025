from setuptools import find_packages, setup

package_name = 'lab5_task1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bot',
    maintainer_email='bot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'feature_detector = lab5_task1.feature_detector:main',
        	'lidar_plot = lab5_task1.lidar_plot:main',
            'line_detector = lab5_task1.line_detector:main',
        ],
    },
)
