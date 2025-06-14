from setuptools import find_packages, setup

package_name = 'bouncing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/bouncing_launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lipedras',
    maintainer_email='lfljp@hotmail.com',
    description='Package of bouncing mission from SAE-Eletroquad',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bouncing_node = bouncing.bouncing_node:main',
            'camera_node = bouncing.camera_node:main'
        ],
    },
)

