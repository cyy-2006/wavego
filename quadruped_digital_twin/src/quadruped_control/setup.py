import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'quadruped_control'

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
    description='Command-side nodes for the quadruped digital twin',
    license='MIT',
    entry_points={
        'console_scripts': [
            'joint_commander = quadruped_control.joint_commander:main',
        ],
    },
)
