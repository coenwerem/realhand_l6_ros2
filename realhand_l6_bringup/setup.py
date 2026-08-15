from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'realhand_l6_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    maintainer='Clinton Enwerem',
    maintainer_email='enwerem@terpmail.umd.edu',
    description='Launch files, controller configuration, contact visualizer, and mock feeders '
                'for the RealHand L6 ros2_control stack.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'contact_viz = realhand_l6_bringup.contact_viz:main',
            'mock_force_ramp = realhand_l6_bringup.mock_force_ramp:main',
            'mock_can_feeder = realhand_l6_bringup.mock_can_feeder:main',
            'can_tactile_probe = realhand_l6_bringup.can_tactile_probe:main',
        ],
    },
)
