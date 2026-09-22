
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur3_letter_h'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hien',
    maintainer_email='hien@todo.todo',
    description='ROS 2 UR3e Letter H Drawing Assignment',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_h_node = ur3_letter_h.draw_h_node:main',
        ],
    },
)
