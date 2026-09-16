from setuptools import find_packages, setup

package_name = 'waregv_driver'

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
    maintainer='milon',
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
            "ydlidar=waregv_driver.ydlidar:main",
            
            "imu=waregv_driver.imu:main",
            "motor_encoder=waregv_driver.motor_encoder:main",
            "controller_link=waregv_driver.controller_link:main"
            
        ],
    },
)
