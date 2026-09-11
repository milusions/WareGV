from setuptools import find_packages, setup

package_name = 'waregv_controller'

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
                        "inverse_kinematics=waregv_controller.inverse_kinematics:main"
        ],
    },
)
