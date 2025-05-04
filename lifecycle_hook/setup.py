from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lifecycle_hook'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas',
    maintainer_email='d2023001147@unifei.edu.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_control_node = lifecycle_hook.mission_control.mission_control_node:main',
            'navigation_node = lifecycle_hook.nav.navigation_node:main',
            'vision_node = lifecycle_hook.vision.vision_node:main',
            'actuator_node = lifecycle_hook.act.actuator_node:main'
            ],
    },
)
