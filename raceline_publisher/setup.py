from setuptools import find_packages, setup
from glob import glob

package_name = 'raceline_publisher'

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
    maintainer_email='ebrahim@example.com',
    description='Standalone publisher of a raceline CSV as WaypointArray/MarkerArray.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'raceline_publisher_node = raceline_publisher.raceline_publisher_node:main',
        ],
    },
)
