from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'arm_2dof_wrist_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='athena',
    maintainer_email='todo@todo.com',
    description='Python replacement for the 2DOF wrist joint-by-joint controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_2dof_wrist_node = arm_2dof_wrist_py.manual_2dof_wrist_node:main',
        ],
    },
)
