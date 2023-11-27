#  Copyright (c) 2021. Lorem ipsum dolor sit amet, consectetur adipiscing elit.
#  Morbi non lorem porttitor neque feugiat blandit. Ut vitae ipsum eget quam lacinia accumsan.
#  Etiam sed turpis ac ipsum condimentum fringilla. Maecenas magna.
#  Proin dapibus sapien vel ante. Aliquam erat volutpat. Pellentesque sagittis ligula eget metus.
#  Vestibulum commodo. Ut rhoncus gravida arcu.

import os
from setuptools import setup


# Utility function to read the README file.
# Used for the long_description.  It's nice, because now 1) we have a top level
# README file and 2) it's easier to type in the README file than to put a raw
# string in below ...
def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    name="pyhelios",
    version="1.3.0",
    author="HELIOS++ dev team",
    author_email="helios@uni-heidelberg.de",
    description=("python bindings of the HELIOS++ Virtual Laser Scanning "
                 "Simulator"),
    license="LGPL",
    keywords="laserscanner simulation lidar",
    url="https://github.com/3dgeo-heidelberg/helios",
    packages=['pyhelios'],
    long_description="",
    classifiers=[
        "Development Status :: 3 - Alpha",
    ],
)
