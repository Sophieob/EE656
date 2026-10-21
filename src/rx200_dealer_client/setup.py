from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rx200_dealer_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yam1')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='master26',
    maintainer_email='master26@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rx200_dealer=rx200_dealer_client.rx200_dealer_client_node:main'
        ],
    },
)
