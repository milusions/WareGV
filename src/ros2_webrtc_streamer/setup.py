from setuptools import setup
from glob import glob

package_name = 'ros2_webrtc_streamer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'webrtc_streamer = ros2_webrtc_streamer.webrtc_streamer:main',
        ],
    },
)
