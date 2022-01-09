import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'yeet_go_run_my_turtle'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gon',
    maintainer_email='gksrmffh1@gmail.com',
    description='Train your turtle',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trainer = yeet_go_run_my_turtle.trainer:main',
        ],
    },
)
