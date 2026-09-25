import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'waregv_user_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files if needed
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='milon',
    maintainer_email='milon@todo.todo',
    description='User interface nodes for WareGV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'nav2_status_node = waregv_user_interfaces.nav2_status_node:main',
           
        ],
    },
)