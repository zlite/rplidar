#!/usr/bin/env python3
'''Records measurments to a given file. Usage example:

$ ./record_measurments.py out.txt'''
import sys
import time
from rplidar import RPLidar
import PyCmdMessenger
import math
angleoffset = 45
gain = 2
speed = 90
left = 0
right = 0

PORT_NAME = '/dev/ttyUSB0' # this is for the Lidar

# Initialize an ArduinoBoard instance.  This is where you specify baud rate and
# serial timeout.  If you are using a non ATmega328 board, you might also need
# to set the data sizes (bytes for integers, longs, floats, and doubles).  
myarduino = PyCmdMessenger.ArduinoBoard("/dev/ttyACM0",baud_rate=9600)

# List of commands and their associated argument formats. These must be in the
# same order as in the sketch.
commands = [["motors","iii"], # motor power, left, right
            ["get_sonar",""], # commands a sonar sample
            ["sonar","i"], # result of sonar sample
            ["sonar_angle","i"], # commands a sonar servo angle
            ["line_tracker","iiii"], # output of line-follower senors
            ["ir_in","f"], # commands from IR remote control
            ["error","s"]]

# Initialize the messenger
c = PyCmdMessenger.CmdMessenger(myarduino,commands)


def steer(angle):
    global speed
    # Send motor commands
    offset = int(20*(math.cos(math.radians(angle))))
    print ("offset: ", offset)
    if offset <= 0:
        left = -1* offset * gain
        right = 0
    else:
        right = offset * gain
        left = 0
    print (speed, left, right) 
    c.send("motors",speed,left,right)
    

def run():
    '''Main function'''
    lidar = RPLidar(PORT_NAME)
    lidar.start_motor()
    time.sleep(1)
    info = lidar.get_info()
    print(info)
    try:
        print('Recording measurments... Press Crl+C to stop.')
        try:
            for measurment in lidar.iter_measurments():
#            line = '\t'.join(str(v) for v in measurment)
#            print(line + '\n')
                if (measurment[2] > 0 and measurment[2] < 90):
                    if (measurment[3] < 1000 and measurment[3] > 100):
                        print ("Angle: ", measurment[2]-angleoffset)
                        print ("Distance: ", measurment[3])
                        steer (measurment[2]-angleoffset)
        except KeyboardInterrupt:
            print('Stopping.')
    except KeyboardInterrupt:
        print('Stopping.')
    lidar.stop()
    lidar.stop_motor()
    c.send("motors",0,0,0)  # turn off wheel motors
    lidar.disconnect()

if __name__ == '__main__':
    run()
