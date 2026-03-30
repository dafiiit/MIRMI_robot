import os
from glob import glob
from setuptools import setup

package_name = 'pole_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Pole detector node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pole_detector = pole_detector.pole_detector:main',
            'pole_marker = pole_detector.pole_marker:main'
        ],
    },
)
