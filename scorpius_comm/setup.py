from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'scorpius_comm'

# ROS2 entry point script uses 'scorpius-comm' as distribution name.
# Keep this aligned to avoid importlib.metadata.PackageNotFoundError.
setup(
    name='scorpius-comm',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='chuck',
    maintainer_email='chucklafond2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'comm = scorpius_comm.comm_node:main',
        ],
    },
)
