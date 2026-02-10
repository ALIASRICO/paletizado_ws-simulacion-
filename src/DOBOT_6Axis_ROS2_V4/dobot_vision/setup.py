from setuptools import setup
import os
from glob import glob

package_name = 'dobot_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@dobot.com',
    description='Perception layer for Dobot CR20 palletizing system',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_publisher = dobot_vision.pose_publisher:main',
            'vision_display = dobot_vision.vision_display:main',
            'calibration_node = dobot_vision.calibration_node:main',
        ],
    },
)
