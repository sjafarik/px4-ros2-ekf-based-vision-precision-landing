from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'drone_precision_landing_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Saeed Jafari Kang',
    maintainer_email='sjafarik@mtu.edu',
    description='ROS2 + PX4 EKF-based precision landing package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = drone_precision_landing_py.offboard_control:main',
            'mission_manager = drone_precision_landing_py.mission_manager:main',
            'pad_detector = drone_precision_landing_py.pad_detector:main',
            'pad_tracker_ekf = drone_precision_landing_py.pad_tracker_ekf:main',
            'landing_manager = drone_precision_landing_py.landing_manager:main',
        ],
    },
)