from setuptools import setup
import os
from glob import glob

package_name = 'patrol_bt_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # THIS LINE installs your BT XML into the share directory
        # so get_package_share_directory() can find it at runtime
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'patrol_bt_node = patrol_bt_pkg.patrol_bt_node:main',
        ],
    },
)