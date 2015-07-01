#!/usr/bin/env python

import rospy                        #for interacting with ROS topics and parameters
import sys, getopt                  #for parameters and sys.exit()
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud2, LaserScan
from pcg_node.msg import LaserScanAngle

import re                           #for findall()
import string                       #for split()
import time

# Change this to change the speed of this program:
hertz = 100

sliceBuffer = []
counter = 0

# initialize ROS node
rospy.init_node('PC_Slice_Node', anonymous=True)

# Declare some ROS topic subscriber callback function
def sliceCallback(msg):
    global sliceBuffer
    global counter
    # Store the slice in the slice buffer
    sliceBuffer.append(msg)
    counter += 1
    print "Message received #%d" % counter
    print msg

# Instantiate the subscribers
scanSubscriber  = rospy.Subscriber("slice", LaserScanAngle, sliceCallback)

# Create a rate control object
rate = rospy.Rate(hertz)

print "Started PC Slice Node at %d hertz." % hertz

while not rospy.is_shutdown():
    # Main loop
    

    rate.sleep()


print "PCG done, waiting until ctrl+c is hit..."
rospy.spin()  # just to prevent this node from exiting