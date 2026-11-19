from setuptools import setup

package_name = 'carla_map_gen'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.repairs'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Autoware Lanelet2 maps from CARLA or an OpenDRIVE file: conversion, georeference, repairs',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'carla_map_gen = carla_map_gen.generate:main',
            'carla_map_repair = carla_map_gen.repair:main',
        ],
    },
)
