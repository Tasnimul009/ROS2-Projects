from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'object_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS2 YOLOv8 Object Detection Node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detection_node = object_detection.object_detection_node:main',
        ],
    },
)
