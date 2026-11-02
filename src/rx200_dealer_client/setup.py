from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rx200_dealer_client'

setup(
    name=package_name,
    version='0.1.0',
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
    maintainer='Maynooth University MSc. Robotics and Embedded AI Group 5',
    maintainer_email='sophie.obrien.2021@mumail.ie',
    description='ROS 2 card-dealing project built around the Interbotix RX200 robotic arm',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rx200_dealer=rx200_dealer_client.rx200_dealer_client_node:main',
            'cli=rx200_dealer_client.cli:main',
        ],
    },
)
