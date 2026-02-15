from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'lab1_amr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'),   glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hero',
    maintainer_email='hero@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'wheel_odom = lab1_amr.wheel_odom:main',
        'ekf_yaw_fusion = lab1_amr.ekf_yaw_fusion:main',
        'icp_odom = lab1_amr.icp_odom:main',
        ],
    },
)
