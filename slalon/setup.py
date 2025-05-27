from setuptools import find_packages, setup

package_name = 'slalon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slalon_launch.py']),
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
            'depth_st = slalon.depth_st:main',
            'movement = slalon.movement:main',
            'test_detection = slalon.test_detection:main'
        ],
        'launch': [
        'slalon_launch = slalon.launch.slalon_launch:generate_launch_description',
    ],
    },
)
