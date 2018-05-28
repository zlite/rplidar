#!/usr/bin/env python

'''
setup.py - Python distutils setup file for BreezyLidar package.

Copyright (C) 2014 Simon D. Levy

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

from distutils.core import setup, Extension


module = Extension('pybreezylidar', 
    sources = [ 
        'pybreezylidar.c', 
        'pyextension_utils.c', 
        '../c/hokuyo.c', 
        '../c/serial_device.c']
    )


setup (name = 'BreezyLidar',
    version = '0.1',
    description = 'Lidar access in Python',
    packages = ['breezylidar'],
    ext_modules = [module],
    author='Simon D. Levy',
    author_email='levys@wlu.edu',
    url='http://home.wlu.edu/~levys/software/breezylidar',
    license='LGPL',
    platforms='Linux; Windows; OS X',
    long_description = 'Provides class URG04LX'
    )
