from setuptools import find_packages, setup

package_name = 'gps_position_publisher'

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
    description='Simple GPS NavSatFix publisher',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_publisher = gps_position_publisher.gps_publisher_node:main',
        ],
    },
)
