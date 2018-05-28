#!/usr/bin/env python

'''
    xvlidar.py - Python class for reading from GetSurreal's XV Lidar Controller.  

    Adapted from lidar.py downloaded from 

      http://www.getsurreal.com/products/xv-lidar-controller/xv-lidar-controller-visual-test

    Copyright (C) 2016 Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
'''

import threading, time, serial, traceback

class XVLidar(object):

    def __init__(self, com_port):
        '''
        Opens a serial connection to the XV Lidar on the specifiec com port (e.g.,
        'COM5', '/dev/ttyACM0').  Connection will run on its own thread.
        '''
        self.ser = serial.Serial(com_port, 115200)
        self.thread = threading.Thread(target=self._read_lidar, args=())
        self.thread.daemon = True
        self.state = 0
        self.index = 0
        self.lidar_data = [()]*360 # 360 elements (distance,quality), indexed by angle
        self.speed_rpm = 0
        self.thread.start()

    def getScan(self):
        '''
        Returns 360 (distance, quality) tuples.
        '''
        return [pair if len(pair) == 2 else (0,0) for pair in self.lidar_data]

    def getRPM(self):
        '''
        Returns speed in RPM.
        '''
        return self.speed_rpm

    def _read_bytes(self, n):

        return self.ser.read(n).decode('ISO-8859-1')
     
    def _read_lidar(self):

        nb_errors = 0

        while True:

            try:

                time.sleep(0.0001) # do not hog the processor power

                if self.state == 0 :
                    b = ord(self._read_bytes(1))
                    # start byte
                    if b == 0xFA :
                        self.state = 1
                    else:
                        self.state = 0
                elif self.state == 1:
                    # position index
                    b = ord(self._read_bytes(1))
                    if b >= 0xA0 and b <= 0xF9 :
                        self.index = b - 0xA0
                        self.state = 2
                    elif b != 0xFA:
                        self.state = 0
                elif self.state == 2 :

                    data = [ord(b) for b in self._read_bytes(20)]

                    # speed
                    b_speed = data[:2]
                    
                    # data
                    b_data0 = data[2:6]
                    b_data1 = data[6:10]
                    b_data2 = data[10:14]
                    b_data3 = data[14:18]

                    # checksum
                    b_checksum = data[18:20]

                    # for the checksum, we need all the data of the packet...
                    # this could be collected in a more elegent fashion...
                    all_data = [ 0xFA, self.index+0xA0 ] + data[:18]

                    # checksum
                    incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)

                    # verify that the received checksum is equal to the one computed from the data
                    if self._checksum(all_data) == incoming_checksum:

                        self.speed_rpm = float( b_speed[0] | (b_speed[1] << 8) ) / 64.0
                        
                        self._update(0, b_data0)
                        self._update(1, b_data1)
                        self._update(2, b_data2)
                        self._update(3, b_data3)

                    else:
                        # the checksum does not match, something went wrong...
                        nb_errors +=1
                        print('Checksum fail')
                        
                        
                    self.state = 0 # reset and wait for the next packet
                    
                else: # default, should never happen...
                    self.state = 0

            except:
                traceback.print_exc()
                exit(0)

    def _update(self, offset, data ):

        angle = self.index * 4 + offset

        #unpack data using the denomination used during the discussions
        x = data[0]
        x1= data[1]
        x2= data[2]
        x3= data[3]
        
        dist_mm = x | (( x1 & 0x3f) << 8) # distance is coded on 13 bits ? 14 bits ?
        quality = x2 | (x3 << 8) # quality is on 16 bits
        self.lidar_data[angle] = dist_mm,quality

    def _checksum(self, data):
        """Compute and return the checksum as an int.
           data -- list of 20 bytes (as ints), in the order they arrived in.
        """
        # group the data by word, little-endian
        data_list = []
        for t in range(10):
            data_list.append( data[2*t] + (data[2*t+1]<<8) )
        
        # compute the checksum on 32 bits
        chk32 = 0
        for d in data_list:
            chk32 = (chk32 << 1) + d

        # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
        checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
        checksum = checksum & 0x7FFF # truncate to 15 bits
        return int( checksum )
