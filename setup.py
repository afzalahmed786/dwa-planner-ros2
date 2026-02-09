import os
from glob import glob
from setuptools import setup

package_name = 'custom_dwa_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='afzal',
    maintainer_email='afzalahmed3012@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dwa_planner = custom_dwa_planner.dwa_planner_node:main',
            'goal_publisher = custom_dwa_planner.goal_publisher:main',
        ],
    },
)
