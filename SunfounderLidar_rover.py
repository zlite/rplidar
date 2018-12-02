#!/usr/bin/env python3

import sys
import time
from rplidar import RPLidar
import math
import Adafruit_PCA9685


# Initialise the PCA9685 servo driver board using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

angle_offset = 0 # this compensates for the Lidar being placed in a rotated position
gain = 3.1 # this is the steering gain. The PWM output to the steering servo must be between 0 (left) and 500 (right)
speed = 2000 # crusing speed, must be between 0 and 3600
steering_correction = 40 # this compensates for any steering bias the car has. Positive numbers steer to the right
start = time.time()
stop = False
left_motor = 4 # which PWM output this is attached to
right_motor = 5
steer_servo = 0
speed_adj = 2 # gain for slowing down in turns

PORT_NAME = '/dev/ttyUSB0' # this is for the Lidar

def constrain(val, min_val, max_val):
    if val < min_val: return min_val
    if val > max_val: return max_val
    return val

def drive(speed):  # speed must be between 0 and 3600
    servo (left_motor,speed)
    servo (right_motor,speed)

def servo (channel, PWM):
    pulse = PWM + 300 # make sure pulse width at least 500
    pwm.set_pwm(channel,0, pulse) # channel, start of wave, end of wave

def steer(angle):
    angle = 250 + steering_correction + gain*angle
    angle = int(constrain(angle,0,600))
    print ("PWM output: ", angle)
    servo (steer_servo,angle)
    new_speed = speed - (speed_adj*abs(angle-100))
    new_speed = constrain(new_speed, 100, 5000)
#    drive (new_speed)
    drive (speed)



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
                drive(0)
                lidar.disconnect()
                break
            if (measurment[2] > 315 or measurment[2] < 45):  # in angular range
                if (measurment[3] < 1000 and measurment[3] > 100): # in distance range
#                    print (measurment[2])
                    if (measurment[2] < 45):
                        temp = measurment[2]
                    else:
                        temp = -1* (360-measurment[2]) # convert to negative angle to the left of center
                    data = data + temp # sum of the detected angles, so we can average later
#                    range_sum = range_sum + measurment[3] # sum all the distances so we can normalize later
                    counter = counter + 1 # increment counter
            if time.time() > (lasttime + 0.2):
#                print("this should happen ten times a second")
                if counter > 0:  # this means we see something
                    average_angle = (data/counter) - angle_offset # average of detected angles
                    print ("Average angle: ", average_angle)
                    obstacle_direction = int(100*math.atan(math.radians(average_angle)))  # convert to a vector component
                    drive_direction = -1 * obstacle_direction # steer in the opposite direction as obstacle (I'll replace this with a PID)
#                    print ("Drive direction: ", drive_direction)
                    counter = 0 # reset counter
                    data = 0  # reset data
                    range_sum = 0
                else:
                    drive_direction = 0
                steer(drive_direction)  # Send data to motors
                lasttime = time.time()  # reset 10Hz timer

def run():
    '''Main function'''
    lidar = RPLidar(PORT_NAME)
    lidar.start_motor()
    servo (steer_servo,150) # center servo
    time.sleep(1)
    info = lidar.get_info()
    print(info)
    try:
        scan(lidar)
    except KeyboardInterrupt:
        stop = True
        print('Stopping.')
        lidar.stop()
        lidar.stop_motor()
        drive (0)
        lidar.disconnect()

if __name__ == '__main__':
    run()
