import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'drive_control'

setup(
    name=package_name,
    version='1.0.0',
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
    maintainer='Timo Fahrmer',
    maintainer_email='timofahrmer@gmail.com',
    description='TurtleBot3 Burger Fahrsteuerung (Python).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = drive_control.drive_node:main',
        ],
    },
)
