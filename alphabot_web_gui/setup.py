import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'alphabot_web_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdll',
    maintainer_email='abdll1@hotmail.com',
    description='Local Network Web GUI for AlphaBot / ServiceBot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gateway_node = alphabot_web_gui.gateway_node:main',
            'sim_robot_node = alphabot_web_gui.sim_robot_node:main',
        ],
    },
)
