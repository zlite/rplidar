#!/usr/bin/env python3
'''Records measurments to a given file. Usage example:

$ ./record_measurments.py out.txt'''
import sys
import time
from rplidar import RPLidar
import PyCmdMessenger
import math

angle_offset = 50 # this compensates for the Lidar being placed in a rotated position
gain = 2.0 # this is the steering gain
speed = 80 # crusing speed
steering_correction = -10 # this compensates for any steering bias the car has. Positive numbers steer to the right
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
                    average_angle = (data/counter) - angle_offset # average of detected angles
                    obstacle_direction = int(100*math.atan(math.radians(average_angle)))  # convert to a vector component
                    drive_direction = -1 * obstacle_direction # steer in the opposite direction as obstacle (I'll replace this with a PID)
                    steer(drive_direction)  # Send data to motors
                    print ("Drive direction: ", drive_direction)
                    counter = 0 # reset counter
                    data = 0  # reset data
                    range_sum = 0
                lasttime = time.time()  # reset 10Hz timer

def steer(angle):
    global speed, gain, stop
    # Send motor commands
    if angle >= 0:  # steer to right
        right_wheels = (-1 * angle * gain) + steering_correction  # slow down right wheels to steer left
        left_wheels = (-1*right_wheels) # do the opposite of what the right does
    else: # steer to left
        right_wheels = (1 * angle * gain) + steering_correction # speed up right wheels to steer right
        left_wheels = (-1*right_wheels)  # do the opposite of what the right does
    print (speed, left_wheels, right_wheels) 
    c.send("motors",speed,int(left_wheels),int(right_wheels))

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
