from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('uav_detection/*.pt')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hubert Szolc',
    maintainer_email='jidzi@agh.edu.pl',
    description='Detekcja UAV z telemetrią mavros (YOLO, QR, zrzut ładunku)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uav_detection_pub = uav_detection.node:main',
            'uav_detection_sub = uav_detection.uav_detection_sub:main',
            'servo_controller = uav_detection.servo_controller:main',
        ],
    },
)
