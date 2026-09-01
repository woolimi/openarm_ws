import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'openarm_leader'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Woolim Park',
    maintainer_email='woolim@pinklab.art',
    description='Feetech 리더암으로 OpenArm v1.0 을 teleoperation 하는 relay 노드와 실습 launch.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'leader_node = openarm_leader.leader_node:main',
            'calibrate = openarm_leader.calibrate:main',
            'check = openarm_leader.check:main',
            'register = openarm_leader.register:main',
            'udev = openarm_leader.udev:main',
        ],
    },
)
