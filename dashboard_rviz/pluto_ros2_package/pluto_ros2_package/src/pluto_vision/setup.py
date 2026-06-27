from setuptools import find_packages, setup

package_name = 'pluto_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@local',
    description='Vision-based control for Pluto.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_follow = pluto_vision.aruco_follow:main',
            'face_tracking = pluto_vision.face_tracking:main',
        ],
    },
)
