from setuptools import find_packages, setup
from glob import glob

package_name = 'adaptive_controller_benchmark'

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
    maintainer_email='ebrahimabdelghfar550@gmail.com',
    description='Academic benchmark of the adaptive_controller_manager PP/MPC switching FSM.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'adaptive_controller_benchmark_node = '
            'adaptive_controller_benchmark.adaptive_controller_benchmark_node:main',
        ],
    },
)
