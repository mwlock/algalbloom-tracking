# from setuptools import find_packages
# from distutils.core import setup


# packages=find_packages()
# package_name = 'smarc_algal_bloom_tracking'

# print(packages)

# setup(
#     version='0.0.0',
#     packages = packages,
# )

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['controller','estimators','publishers','services'],
    package_dir={'': 'src'}
)

setup(**d)