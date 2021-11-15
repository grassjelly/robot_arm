import os
from glob import glob
from setuptools import setup

package_name = 'robot_arm_apps'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'shapely'],
    zip_safe=True,
    maintainer='Juan Miguel jimeno',
    maintainer_email='jimenojmm@gmail.com',
    description='ROS2 Robot Arm Driver',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'pick_n_place = robot_arm_apps.pick_n_place:main'
        ],
    },
)
