from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rob2002_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grzegorz Cielniak',
    maintainer_email='gcielniak@lincoln.ac.uk',
    description='The tutorial package containing basic functional components for the ROB2002 module.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mover_basic = rob2002_tutorial.mover_basic:main',
            'mover_laser = rob2002_tutorial.mover_laser:main',
            'mover_spinner = rob2002_tutorial.mover_spinner:main',
            'mover_waypoints = rob2002_tutorial.mover_waypoints:main',
            'detector_basic = rob2002_tutorial.detector_basic:main',
            'detector_better = rob2002_tutorial.detector_dblcounting:main',
            'detector_3d = rob2002_tutorial.detector_3d:main',
            'counter_basic = rob2002_tutorial.counter_basic:main',
            'counter_3d = rob2002_tutorial.counter_3d:main',
            'wander_detect = rob2002_tutorial.wander:main'
        ],
    },
)
