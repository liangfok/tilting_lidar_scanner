#!/usr/bin/env python

import rospy                        #for interacting with ROS topics and parameters
import sys, getopt                  #for parameters and sys.exit()
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud2, LaserScan

import re                           #for findall()
import string                       #for split()
import serial
import time

ser = 0
inputStringBuffer = ""

# initialize ROS node
rospy.init_node('Tilting_Stand_Node', anonymous=True)

# instantiate publisher
anglePublisher = rospy.Publisher("angle", Float64, queue_size = 0)

# instantiate messages to publish
angleMsg = Float64()

stepCommand = None

def stepCallback(msg):
    global stepCommand
    print "TSN: stepCallback called, step: {0}".format(msg.data)
    stepCommand = msg.data

# instantiate the subscribers
stepSubscriber = rospy.Subscriber("step", Int32, stepCallback)

def readEntireLine():
    global ser
    global inputStringBuffer

    while True:
        data = ser.readline()
        if data:
            inputStringBuffer += data

            if "\n" in inputStringBuffer:
                indexOfReturn = inputStringBuffer.index("\n")
                
                # Extract the data from the beginning of the string
                # buffer to the location of the carrage return
                result = inputStringBuffer[0:indexOfReturn + 1]

                # Remove the result from the string buffer
                inputStringBuffer = inputStringBuffer[indexOfReturn + 1:]
                return result

# Code that opens a serial port connection with the Arduino
def openSerialPort():
    global ser

    # Open a serial connection with the Arduino
    # IF ARDUINO DOESN'T CONNECT, CHANGE  THIS (could be ACM0 or ACM2)
    #                          |
    #                          |
    #                          V
    ser = serial.Serial('/dev/ttyACM1', 9600, timeout = .01)

    # Clear the serial port's input buffer so we don't receive
    # any garbage during first read
    ser.flushInput() 
    
    print "Connecting..."
    time.sleep(2)
    print "Connected"
    
    if ser.isOpen():
        print "Port Open"
        return True
    else:
        return False

def initArduino():
    global ser

    ser.write('5\n')
    # ser.flush()
    
    print "stepAngle: wrote to serial: 5"
    
    # time.sleep(.1)
    done = False
    while not done:
        data = readEntireLine() #[:-2] #the last bit gets rid of the new-line chars
        print "stepAngle: Recieved data: \"{0}\"".format(data)
        if data[len(data)-3:-2] == "T":
            print "stepAngle: Got confirmation that Arduino started!"
            done = True

        time.sleep(0.1) # 10Hz

    return True

def takeOneStep():
    global ser
    
    stepCounter = 0

    # Tell the stepper motor to take a step
    # This is signified by a 3
    ser.write('3\n')
    print "stepAngle: Python told Arduino to move one step"
    
    done = False
    while not done:
        data = readEntireLine() #the last bit gets rid of the new-line chars
        if data:
            print "stepAngle: Recieved data: \"{0}\"".format(data)
            if data[len(data)-3:-2] == "F":
                print "stepAngle: finished moving one step"
                #time.sleep(.1)
                done = True

if openSerialPort():
    if initArduino():
        #continue

        rate = rospy.Rate(1) # 1Hz

        while not rospy.is_shutdown():

            # Check if we've received the angle and laser scan data
            if stepCommand == None:
                print "TSN: Did not receive step command yet..."
            else:
                print "TSN: The step command is {0}".format(stepCommand)

                # TODO: Communicate with the Arduino. Tell it to take a step and
                # get the latest angle of the tilting base.

                takeOneStep()

                currentAngle = 0.0  # TEMP!  Replace Me!

                stepCommand = None # So we can detect when a new step command arrives.

                # TODO: Save the current angle into the angleMsg
                angleMsg.data = currentAngle
                anglePublisher.publish(angleMsg)

            rate.sleep()

print "TSN done, waiting until ctrl+c is hit..."
rospy.spin()  # just to prevent this node from exiting