from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'test_experiments'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='michele.pierallini@gmail.com',
    description='Simple Test for the Fishingrod',
    license='License declaration',
    # tests_require=['pytest'],
    extras_require={
    'test': ['pytest'],
            },
    entry_points={
        'console_scripts': [
            'test_node = test_experiments.test_node:main',
        ],
    },
)

