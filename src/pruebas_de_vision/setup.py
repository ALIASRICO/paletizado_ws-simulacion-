import os
from glob import glob
from setuptools import setup

package_name = 'pruebas_de_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include worlds
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Paquete de pruebas de visión para cámara Gazebo',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_viewer = pruebas_de_vision.camera_viewer:main',
            'yolo_detector = pruebas_de_vision.yolo_detector:main',
            'train_yolo = pruebas_de_vision.train_yolo:main',
        ],
    },
)
