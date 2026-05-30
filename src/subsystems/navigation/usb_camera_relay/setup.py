from setuptools import setup
from glob import glob
import os

package_name = 'usb_camera_relay'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UMD Loop',
    maintainer_email='todo@todo.com',
    description='Publishes USB camera streams as sensor_msgs/Image ROS topics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_relay = usb_camera_relay.camera_relay_node:main',
        ],
    },
)
