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
            # Test executables
            'test_a_distance = docking_test_suite.test_a_distance:main',
            'test_b_angular = docking_test_suite.test_b_angular:main',
            'test_c_dynamic = docking_test_suite.test_c_dynamic:main',
            'test_d_environmental = docking_test_suite.test_d_environmental:main',

            # Debug / utility tools
            'robot_driver_cli = docking_test_suite.robot_driver:main',
            'docking_diagnostics = docking_test_suite.diagnostics:main',
            'gdrive_upload = docking_test_suite.gdrive_uploader:main',
            'docking_analyze = docking_test_suite.analysis:main',
            # TF utilities
            'pose_tf_broadcaster = docking_test_suite.pose_tf_broadcaster:main',
        ],
    },
)
