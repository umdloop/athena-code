import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'umdloop_can'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools',],
    zip_safe=True,
    maintainer='Adrian Cires',
    maintainer_email='loop@adriancires.com',
    description='ROS2 CAN Package for UMD Loop',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'can_node = umdloop_can.can_node:main',
        ],
    },
)
