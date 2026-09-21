import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'waregv_controller'


def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for folder in ['launch']:
    for file_path in package_files(folder):
        install_dir = os.path.join('share', package_name, os.path.dirname(file_path))
        data_files.append((install_dir, [file_path]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='milon paul jose',
    maintainer_email='milonpauljs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "joystick_control=waregv_controller.joystick_controller:main",
                        "imu_chasis=waregv_controller.imu_chasis:main"
        ],
    },
)
