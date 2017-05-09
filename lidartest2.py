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
        print ("Quality:", np.array(scan[1][0]))
        print ("Angle: ", np.array(scan[1][1]))
        print ("Distance:", np.array(scan[1][2]))
except KeyboardInterrupt:
    print("Stopping...")
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
