from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'exo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sungbincho',
    maintainer_email='sungbincho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_driver_node = exo_control.motor_driver_node:main',
            'motor_control_node = exo_control.motor_control_node:main',
            'gpe_driver_node = exo_control.gpe_driver_node:main',
            'gpe_control_node = exo_control.gpe_control_node:main',
            'gpe_torque_profile_node = exo_control.gpe_torque_profile_node:main',
        ],
    },
)
