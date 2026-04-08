from setuptools import find_packages, setup

package_name = 'station_detection_APRILTAG'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/APRILTAG_detection.launch.py']),
        ('share/' + package_name + '/config', ['config/tags.yaml', 'config/isaac_ros_apriltag_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='holybro',
    maintainer_email='david.metzler.2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'apriltag_visualizer = station_detection_APRILTAG.apriltag_visualizer:main',
            'depth_visualizer = station_detection_APRILTAG.depth_visualizer:main',
            'accuracy_test = station_detection_APRILTAG.accuracy_test:main',
        ],
    },
)
