from setuptools import find_packages
from setuptools import setup

setup(
    name='pi3hat_moteus_int_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('pi3hat_moteus_int_msgs', 'pi3hat_moteus_int_msgs.*')),
)
