import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'quadruped_twin'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Digital twin sync bridge between physical robot and Isaac Sim mirror',
    license='MIT',
    entry_points={
        'console_scripts': [
            'twin_bridge = quadruped_twin.twin_bridge:main',
        ],
    },
)
