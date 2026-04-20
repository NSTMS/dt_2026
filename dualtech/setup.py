from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'dualtech'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name), glob('dualtech/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hubert Szolc',
    maintainer_email='jidzi@agh.edu.pl',
    description='Package for the DUAL-TECH AGH competition',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uav_detection_pub = dualtech.uav_detection_pub:main',
            'uav_detection_sub = dualtech.uav_detection_sub:main',
            'ugv_detection_pub = dualtech.ugv_detection_pub:main',
            'ugv_detection_sub = dualtech.ugv_detection_sub:main',
	    'servo_controller = dualtech.servo_controller:main',
       ],
    },
)
