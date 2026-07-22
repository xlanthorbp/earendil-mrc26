import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'earendil_bot'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berkay',
    maintainer_email='berkaypaksoy07@gmail.com',
    description='Autonomous navigation and sensor fusion stack for Earendil Bot (MRC 2026).',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hardware_bridge = earendil_bot.bridge.hardware_bridge:main',
            'gps_node = earendil_bot.gps.gps_node:main',
            'aruco_detector = earendil_bot.scripts.aruco_detector:main',
            'gps_nav_main = earendil_bot.scripts.gps_nav_main:main',
            'path_recorder = earendil_bot.scripts.path_recorder:main',
            'gps_nav_test = earendil_bot.tests.gps_nav_test:main',
            'timed_nav_test = earendil_bot.scripts.timed_nav_test:main',
            'heading_test = earendil_bot.tests.heading_test:main',
            'hardware_check = earendil_bot.hardware_check:main',
        ],
    },
)