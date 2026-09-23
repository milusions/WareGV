import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'waregv_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='EKF-fused wheel + gyro odometry',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = waregv_odometry.wheel_odometry_node:main',
            'odom_euler_node = waregv_odometry.odom_euler_node:main',
            "yaw_logger=waregv_odometry.yaw_logger:main",
        ],
    },
)