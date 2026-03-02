from setuptools import setup

package_name = 'carla_pilot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Autonomous driving pilot and pose reader nodes for CARLA-Autoware',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'drive = carla_pilot.drive_node:main',
            'read_poses = carla_pilot.read_poses_node:main',
        ],
    },
)
