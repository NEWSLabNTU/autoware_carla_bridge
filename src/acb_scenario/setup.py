from setuptools import setup

package_name = 'acb_scenario'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/single_vehicle_scenario.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='CARLA scenario scripts for the Autoware-CARLA bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'demo_scenario = acb_scenario.demo_scenario:main',
        ],
    },
)
