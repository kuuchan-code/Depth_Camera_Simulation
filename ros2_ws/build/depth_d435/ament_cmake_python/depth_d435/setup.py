from setuptools import find_packages
from setuptools import setup

setup(
    name='depth_d435',
    version='0.1.0',
    packages=find_packages(
        include=('depth_d435', 'depth_d435.*')),
)
