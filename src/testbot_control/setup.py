from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'testbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'differential_drive_controller = testbot_control.differential_drive_controller:main',
            'odometry_publisher = testbot_control.odometry_publisher:main',
        ],
    },
)
