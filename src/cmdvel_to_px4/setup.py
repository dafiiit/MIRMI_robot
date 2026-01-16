from setuptools import find_packages, setup

package_name = 'cmdvel_to_px4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/px4_bridge.launch.py',
            'launch/startup_combined.launch.py',
        ]),
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
             'cmdvel_to_px4 = cmdvel_to_px4.cmdvel_node:main',
             'cmdvel_to_px4_2 = cmdvel_to_px4.cmdvel_node2:main',
             'cmdvel_to_px4_3 = cmdvel_to_px4.cmdvel_node3:main',
        ],
    },
)
