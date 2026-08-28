from glob import glob

from setuptools import find_packages, setup

package_name = 'trajectory_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fengye_cav',
    maintainer_email='fengye_cav@example.com',
    description='Offline and ROS 2 racing trajectory planner with RViz visualization.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'plan_trajectory = trajectory_planner.cli:main',
            'trajectory_planner_node = trajectory_planner.node:main',
        ],
    },
)
