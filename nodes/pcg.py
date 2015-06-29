#!/usr/bin/env python

import rospy                        #for interacting with ROS topics and parameters
import sys, getopt                  #for parameters and sys.exit()
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud2, LaserScan

import re                           #for findall()
import string                       #for split()

#test commit

# initialize ROS node
rospy.init_node('PCG', anonymous=True)

# instantiate publisher
stepPublisher = rospy.Publisher("step", Int32, queue_size = 0)
cloudPublisher = rospy.Publisher("pointCloud", PointCloud2, queue_size = 0)

# instantiate messages to publish
stepMsg = Int32()
cloudMsg = PointCloud2()

# Explicitly declare these global variables (probably not necessary)
currentAngle = None
currentScan = None

# Declare some ROS topic subscriber callback function
def scanCallback(msg):
    print "PCG: scanCallback called"
    currentScan = msg

def angleCallback(msg):
    print "PCG: angleCallback called"
    currentAngle = msg


# Instantiate the subscribers
scanSubscriber  = rospy.Subscriber("scan", LaserScan, scanCallback)
angleSubscriber = rospy.Subscriber("angle", Float64, angleCallback)

# Create a rate control object
rate = rospy.Rate(1) # 1Hz

while not rospy.is_shutdown():

    # Check if we've received the angle and laser scan data
    if currentAngle == None:
        print "PCG: Did not receive current angle yet..."
    else:
        print "PCG: The current angle is {0}".format(currentAngle.data)

    if currentScan == None:
        print "PCG: Did not receive current scan yet..."
    else:
        print "PCG: The current scan is {0}".format(currentScan)

    if currentAngle != None and currentScan != None:
        print "PCG: Got both laser scan data and angle data!"
        # TODO: Store this information in a data structure
        # Check if we've received a full point cloud scan
        # (for each angle between 0 and 90 degrees, we got a laser scan)
        # If we got the full point cloud scan, do some math to
        # generate the actual point cloud message


    stepPublisher.publish(stepMsg)

    cloudPublisher.publish(cloudMsg)

    rate.sleep()


print "PCG done, waiting until ctrl+c is hit..."
rospy.spin()  # just to prevent this node from exiting