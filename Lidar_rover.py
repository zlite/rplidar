#!/usr/bin/env python3
'''Records measurments to a given file. Usage example:

$ ./record_measurments.py out.txt'''
import sys
import time
from rplidar import RPLidar
import PyCmdMessenger
import math

angle_offset = 50
gain = 2.0
speed = 100
left = 0
right = 0
start = time.time()
stop = False

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


def scan(lidar):
    global stop
    time1 = time.time()
    while True:
        counter = 0
        print('Recording measurements... Press Crl+C to stop.')
        data = 0
        range_sum = 0
        lasttime = time.time()
        for measurment in lidar.iter_measurments():
            if stop == True:
                lidar.stop()
                lidar.stop_motor()
                c.send("motors",0,0,0)  # turn off wheel motors
                lidar.disconnect()
                break
            if (measurment[2] > 0 and measurment[2] < 90):  # in angular range
                if (measurment[3] < 1000 and measurment[3] > 100): # in distance range
                    data = data + measurment[2] # angle weighted by distance; basically we're coming up with an obstacle centroid
#                    range_sum = range_sum + measurment[3] # sum all the distances so we can normalize later
                    counter = counter + 1 # increment counter
            if time.time() > (lasttime + 0.1):
#                print("this should happen ten times a second")
                if counter > 0:  # this means we see something
                    average_angle = (data/counter) # average of detected angles
                    angle = average_angle - angle_offset # compensate for the Lidar being 50 degrees rotated
                    drive(angle)
                    print ("Angle: ", angle)
                    counter = 0 # reset counter
                    data = 0  # reset data
                    range_sum = 0
                lasttime = time.time()  # reset 10Hz timer

def drive(angle):
    global speed, gain, stop
#    print("drive")
    # Send motor commands
    steer = int(100*math.atan(math.radians(angle)))
    print ("steer: ", steer * gain)
    if steer <= 0:
        right = -1 * steer * gain
        left = 0
    else:
        left = steer * gain
        right = 0
    print (speed, left, right) 
    c.send("motors",speed,int(left),int(right))

def run():
    '''Main function'''
    lidar = RPLidar(PORT_NAME)
    lidar.start_motor()
    time.sleep(1)
    info = lidar.get_info()
    c.send("motors",speed,0,0) # start by going forward
    print(info)
    try:
        scan(lidar)
    except KeyboardInterrupt:
        stop = True
        print('Stopping.')
        lidar.stop()
        lidar.stop_motor()
        c.send("motors",0,0,0)  # turn off wheel motors
        lidar.disconnect()
 
if __name__ == '__main__':
    run()
