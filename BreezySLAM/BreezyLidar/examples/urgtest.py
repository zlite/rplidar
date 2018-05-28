#!/usr/bin/env python3

'''
urtest.py : Python test program for BreezyLidar on Hokuyo URG-04LX
             
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

from breezylidar import URG04LX

import sys
from time import time

DEVICE = '/dev/ttyACM0'
NITER  = 10

laser = URG04LX(DEVICE)

print('===============================================================')
print(laser)
print('===============================================================')


start_sec = time()

count = 0

for i in range(1, NITER+1):
    
    sys.stdout.write('Iteration: %3d: ' % i)

    data = laser.getScan()
    
    if data:
        
        print('Got %3d data points' % len(data))
        
        count += 1
        
    else:
        
        print('=== SCAN FAILED ===')
        
elapsed_sec = time() - start_sec

print('%d scans in %f seconds = %f scans/sec' % (count, elapsed_sec, count/elapsed_sec))
        
