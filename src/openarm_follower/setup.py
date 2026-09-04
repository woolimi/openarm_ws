import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'openarm_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Woolim Park',
    maintainer_email='woolim@pinklab.art',
    description='OpenArm v1.0 팔로워 bringup 과 중력보상 캘리브레이션 CLI.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'calibrate = openarm_follower.calibrate:main',
        ],
    },
)
