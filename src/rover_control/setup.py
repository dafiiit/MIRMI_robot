from setuptools import find_packages, setup

package_name = 'rover_control'

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
    description='Rover control nodes (circle cmd_vel + mux)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'circle_cmd_vel_px4 = rover_control.circle_cmd_vel_px4:main',
            'cmd_vel_mux_simple = rover_control.cmd_vel_mux_simple:main',
            'gps_position_pub = rover_control.gps_position_pub:main',
        ],
    },
)
