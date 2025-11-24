from setuptools import find_packages, setup

package_name = 'px4_localization_bridge'

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
    maintainer='holybro',
    maintainer_email='holybro@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fake_odometry_pub = px4_localization_bridge.fake_odometry_pub:main',
            'odom_to_px4 = px4_localization_bridge.odom_to_px4:main',
            'fake_odometry_pub_2 = px4_localization_bridge.fake_pub:main',
        ],
    },
)
