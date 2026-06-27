from setuptools import setup
from glob import glob
import os

package_name = 'pluto_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates', glob('pluto_dashboard/templates/*.html')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@local',
    description='Pluto dashboard',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dashboard = pluto_dashboard.dashboard_node:main',
            'crazyflie_bridge = pluto_dashboard.crazyflie_bridge_node:main',
        ],
    },
)
