from setuptools import setup
import os
from glob import glob

package_name = 'gps_ema_filter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Required for ROS2 package indexing
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stv',
    maintainer_email='stv@todo.todo',
    description='GPS EMA filtering node for OutNav system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ema_filter_node = gps_ema_filter.ema_filter_node:main',
            'motion_filter_node = gps_ema_filter.motion_filter_node:main',
	    'heading_kalman_fusion = gps_ema_filter.heading_kalman_fusion:main',
	    'mag_declination_updater = gps_ema_filter.mag_declination_updater:main',
        ],
    },
)
