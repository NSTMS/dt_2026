from setuptools import setup

package_name = 'qr_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyzbar'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='QR reader node',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'qr_node = qr_reader.qr_node:main',
            'qr_pub = qr_reader.qr_publisher:main',
        ],
    },
)