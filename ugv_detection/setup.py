from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ugv_detection'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('ugv_detection/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hubert Szolc',
    maintainer_email='jidzi@agh.edu.pl',
    description='Detekcja UGV bez telemetrii nawigacyjnej (YOLO, QR)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ugv_detection = ugv_detection.node:main',
            'ugv_detection_pub = ugv_detection.node:main',
            'ugv_detection_sub = ugv_detection.ugv_detection_sub:main',
        ],
    },
)
