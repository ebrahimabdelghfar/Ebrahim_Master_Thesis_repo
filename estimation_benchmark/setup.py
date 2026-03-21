from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'estimation_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ebrahim',
    maintainer_email='ebrahim@local',
    description='Multi-step estimation benchmarking for identified vehicle dynamics models.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'estimation_benchmark_node = estimation_benchmark.estimation_benchmark_node:main',
        ],
    },
)
