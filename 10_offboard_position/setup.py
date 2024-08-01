from setuptools import find_packages, setup

package_name = 'offboard_position'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dobrea Dan-Marius',
    maintainer_email='mdobrea@gmail.com',
    description='Package used to control a UAV running PX4 autopilot based on position approach',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
              'position = offboard_position.position_AMD_AIcontest:main',
        ],
    },
)
