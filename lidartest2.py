import sys
from rplidar import RPLidar
import numpy as np
lidar = RPLidar('/dev/ttyUSB0')

info = lidar.get_info()
print(info)

data = []
health = lidar.get_health()
print(health)

try:
    print('Recording measurements... Press Crl+C to stop.')
    for scan in lidar.iter_scans():
        data.append(np.array(scan))
        print ("Array 0")
        print (np.array(scan[0]))
        print ("Array 1")
        print (np.array(scan[1]))
except KeyboardInterrupt:
    print('Stopping.')
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
