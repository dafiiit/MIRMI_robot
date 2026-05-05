from setuptools import find_packages, setup
import os
import glob

package_name = 'docking_test_suite'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch',
            glob.glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'pyyaml',
        'numpy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='holybro',
    maintainer_email='david.metzler.2003@gmail.com',
    description='Isolated testing package for docking AprilTag accuracy evaluation',
    license='MIT',
    extras_require={
        'test': ['pytest'],
        'gdrive': [
            'google-api-python-client',
            'google-auth-httplib2',
            'google-auth-oauthlib',
        ],
        'analysis': [
            'pandas',
            'matplotlib',
        ],
    },
    entry_points={
        'console_scripts': [
            # Sensor sweep test (topic-driven, no robot driving)
            'sensor_sweep = docking_test_suite.sensor_sweep_node:main',
        ],
    },
)
