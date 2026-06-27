#!/usr/bin/env python3

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pluto_camera_sense'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=['pluto_camera_sense', 'pluto_camera_sense.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='PlutoCam drone camera publisher – streams H.264 from the drone camera over WiFi and publishes on /plutocamera/image_raw.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plutocam_publisher = pluto_camera_sense.plutocam_publisher:main',
        ],
    },
)
