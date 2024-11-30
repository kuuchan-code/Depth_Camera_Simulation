from setuptools import find_packages
from setuptools import setup

setup(
    name='xema_s',
    version='0.1.0',
    packages=find_packages(
        include=('xema_s', 'xema_s.*')),
)
