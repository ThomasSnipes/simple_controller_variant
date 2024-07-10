import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'target_pid_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['target_pid_controller.Command', 'target_pid_controller.Thread', 'target_pid_controller.Video'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*yaml'))),
        (os.path.join('share', package_name, 'config', 'rviz'),
            glob(os.path.join('config', 'rviz', '*rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sdc',
    maintainer_email='sdc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_controller = target_pid_controller.pid_controller_node:main',
            'tf_broadcaster = target_pid_controller.tf_updater_node:main',
            'robot_controller = target_pid_controller.pid_controller_node_sub:main'
        ],
    },
)