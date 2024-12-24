from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
        ('share/' + package_name + '/msg', glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='Max',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_nav = aruco_nav.aruco_nav_node:main',
        ],
    },
    package_data={'aruco_nav': [os.path.join('srv', '*.srv'), os.path.join('msg', '*.msg')]},
)

