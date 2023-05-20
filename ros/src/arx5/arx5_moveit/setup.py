from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
        packages=['arx5_control'],
        scripts=[],
        package_dir={'': 'pkgs'}
        )

setup(**d)