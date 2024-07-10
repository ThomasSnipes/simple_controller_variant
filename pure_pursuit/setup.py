import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
            'path_tracker = pure_pursuit.purepursuit_node:main',
            'variable_speed = pure_pursuit.variable_speed_node:main',
            'tf_broadcaster = pure_pursuit.tf_updater_node:main',
            'testpath_pub = pure_pursuit.test_purepursuit_node:main'
        ],
    },
)
