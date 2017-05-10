#!/usr/bin/env python3
'''Records measurments to a given file. Usage example:

$ ./record_measurments.py out.txt'''
import sys
import time
from rplidar import RPLidar


PORT_NAME = '/dev/ttyUSB0'


def run():
    '''Main function'''
    lidar = RPLidar(PORT_NAME)
    lidar.start_motor()
    time.sleep(1)
    info = lidar.get_info()
    print(info)
    try:
        print('Recording measurments... Press Crl+C to stop.')
        for measurment in lidar.iter_measurments():
#            line = '\t'.join(str(v) for v in measurment)
#            print(line + '\n')
            print ("Angle: ", measurment[2])
            print ("Distance: ", measurment[3])
            if (measurment[2] > 0 and measurment[2] < 90):
                if measurment[3] < 1000:
                    print ("Hit!")
    except KeyboardInterrupt:
        print('Stopping.')
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()

if __name__ == '__main__':
    run()
