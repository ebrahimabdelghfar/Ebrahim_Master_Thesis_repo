from setuptools import find_packages, setup

package_name = 'tire_force_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tire_force_benchmark.launch.py']),
        ('share/' + package_name + '/config', ['config/benchmark_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ebrahim',
    maintainer_email='ebrahim@local',
    description='ROS2 node to benchmark estimated tire forces against IPG CarMaker ground truth.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tire_force_benchmark_node = tire_force_benchmark.tire_force_benchmark_node:main',
        ],
    },
)