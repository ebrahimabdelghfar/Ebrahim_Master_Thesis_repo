from setuptools import find_packages, setup
from glob import glob

package_name = 'pacejka_identification'

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
    install_requires=['setuptools', 'pyro-ppl'],
    zip_safe=True,
    maintainer='Ebrahim',
    maintainer_email='ebrahim@local',
    description='Direct Pacejka Magic Formula coefficient identification from CARLA ground-truth per-wheel tire telemetry.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collector_node = pacejka_identification.data_collector_node:main',
            'identification_node = pacejka_identification.identification_node:main',
        ],
    },
)
