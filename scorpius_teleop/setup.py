from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'scorpius_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anibal',
    maintainer_email='anibal.arango@usherbrooke.ca',
    description='Package for teleoperations of ScorpiUS',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop = scorpius_teleop.teleop_node:main'
        ],
    },
)
