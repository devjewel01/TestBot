import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'testbot_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.rviz')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orbitax',
    maintainer_email='jewel.nath@orbitax.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_processor = testbot_sensors.lidar_processor:main',
            'lidar_visualizer = testbot_sensors.lidar_visualizer:main',
            'lidar_monitor = testbot_sensors.lidar_monitor:main',
            'lidar_test = testbot_sensors.lidar_test:main'
        ],
    },
)