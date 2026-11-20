from setuptools import find_packages
from setuptools import setup

setup(
    name='arx_chassis_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('arx_chassis_interfaces', 'arx_chassis_interfaces.*')),
)
