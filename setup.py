from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tiago_finder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),

        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),

        (os.path.join('share', package_name, 'maps'),
         glob(os.path.join('maps', '*.*'))),

        (os.path.join('share', package_name, 'rviz'),
         glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bsosik',
    maintainer_email='bartek-sosik@wp.pl',
    description='Tiago object finder',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = tiago_finder.my_node:main'
        ],
    },
)
