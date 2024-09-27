import os											# !!!!! DDM
from glob import glob								# !!!!! DDM
from setuptools import find_packages, setup

package_name = 'kamikaze_uav'
submodules   = 'kamikaze_uav/submodules'            # !!!!! DDM

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, submodules],            # !!!!! DDM  
    # packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', 
			['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))		# !!!!! DDM
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dobrea Dan-Marius',
    maintainer_email='mdobrea@gmail.com',
    description='shieldUAV code',
    license='3-Clause BSD license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'videoNode     = kamikaze_uav.video:main',
            'broadcastNode = kamikaze_uav.broadcast:main',
            'detectionNode = kamikaze_uav.detection:main',
            'velocityNode  = kamikaze_uav.velocity:main',
            'controlNode   = kamikaze_uav.control:main',
        ],
    },
)
