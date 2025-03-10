from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'circle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='storagy',
    maintainer_email='storagy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_handler = circle.mission_handler:main',
            'arrival_listener = circle.arrival_listener:main',
            'send_goal = circle.send_goal:main',
            'detect_aruco = circle.detect_aruco:main',
            'aruco_dock = circle.aruco_dock:main',
            'docking = circle.docking:main',
            'rotate90 = circle.rotate90:main',
            # 'navigation_client = circle.navigation_client:main',
            # 'base_BasicNavigator = circle.base_BasicNavigator:main',
        ],
    }
)
