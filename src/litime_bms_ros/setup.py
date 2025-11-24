from setuptools import find_packages, setup

package_name = 'litime_bms_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lennart',
    maintainer_email='lennart.troesken@tum.de',
    description='LiTime BMS BLE interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'litime_bms_ros_node = litime_bms_ros.litime_bms_ros_node:main',
        ],
    },
)
