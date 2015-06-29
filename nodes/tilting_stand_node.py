#!/usr/bin/env python

import rospy                        #for interacting with ROS topics and parameters
import sys, getopt                  #for parameters and sys.exit()
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud2, LaserScan

import re                           #for findall()
import string                       #for split()

# initialize ROS node
rospy.init_node('Tilting_Stand_Node', anonymous=True)

# instantiate publisher
anglePublisher = rospy.Publisher("angle", Float64, queue_size = 0)

# instantiate messages to publish
angleMsg = Float64()

stepCommand = None

def stepCallback(msg):
    print "TSN: stepCallback called, step: {0}".format(msg.data)
    stepCommand = msg.data

# instantiate the subscribers
stepSubscriber = rospy.Subscriber("step", Int32, stepCallback)


# TODO: Add code that opens a serial port connection with the Arduino
# .....

rate = rospy.Rate(1) # 1Hz

while not rospy.is_shutdown():

    # Check if we've received the angle and laser scan data
    if stepCommand == None:
        print "TSN: Did not receive step command yet..."
    else:
        print "TSN: The step command is {0}".format(stepCommand)

        # TODO: Communicate with the Arduino. Tell it to take a step and
        # get the latest angle of the tilting base.

        currentAngle = 0.0  # TEMP!  Replace Me!

        stepCommand = None # So we can detect when a new step command arrives.

        # TODO: Save the current angle into the angleMsg
        angleMsg.data = currentAngle
        anglePublisher.publish(angleMsg)

    rate.sleep()

print "TSN done, waiting until ctrl+c is hit..."
rospy.spin()  # just to prevent this node from exiting