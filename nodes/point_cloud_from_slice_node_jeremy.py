#!/usr/bin/env python

import rospy                        #for interacting with ROS topics and parameters
import sys, getopt                  #for parameters and sys.exit()
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud2, LaserScan
from pcg_node.msg import PointCloudSliceMsg
from std_msgs.msg import Header

import re                           #for findall()
import string                       #for split()
import time
import math

import point_cloud_message_creator

import threading

# Declare a mutex to prevent two threads from accessing
# the sliceBuffer

sliceBuffMutex = threading.Lock()

# Change this to change the speed of this program:
CYCLE_FREQUENCY = 100

sliceBuffer = []

currentSlice = 0
currentSliceString = ""
currentRanges = ""
currentRangePoint = 0
currentAngle = 0

# Data for calculating the actual distance
radius = 0.08         # 0.08 meters (80 mm)
axleHeight = 0.1075   # 0.1075 meters (107.5mm)\
scanRange = 240       # 240 degree scan range

# initialize ROS node
rospy.init_node('PC_Slice_Node', anonymous=True)

# Instantiate a publisher for the sensor_msgs.PointCloud2 message
cloudPublisher = rospy.Publisher("pointCloud", PointCloud2, queue_size = 0)

points = []

def sliceCallback(msg):
    '''
    The callback method for LaserScanAngle point cloud slice subscriptions.
    '''
    global sliceBuffer
    global sliceBuffMutex

    sliceBuffMutex.acquire()
    try:
        # Store the slice in the slice buffer
        sliceBuffer.append(msg)
    finally:
        sliceBuffMutex.release()

# Instantiate the subscribers
scanSubscriber  = rospy.Subscriber("slice", PointCloudSliceMsg, sliceCallback)

def cos(angle):
    rad = math.radians(angle)
    cosVal = math.cos(rad)
    return cosVal

def sin(angle):
    rad = math.radians(angle)
    sinVal = math.sin(rad)
    return sinVal

def pythag(dist1, dist2):
    d1 = math.pow(dist1, 2)
    d2 = math.pow(dist2, 2)
    result = math.sqrt(d1 + d2)
    return result

def findRanges(origData):
    global currentRanges
    beg = "["
    end = "]"

    ran = origData.find(beg)
    ran2 = origData.find(end)

    currentRanges = origData[ran + 1:ran2]

def findAngle(origData):
    global currentAngle

    beg = "tiltAngle:"

    ang = origData.find(beg)
    ang2 = len(origData)

    # Cut the string off so that we only get the angle and not stepAngle
    currentAngle = origData[ang + 11:ang2]

def findScanXY(dist, scanAngle):
    if scanAngle < 30:
        newScanAng = 30 - scanAngle
        x = dist * sin(newScanAng) * -1
        y = dist * cos(newScanAng)
    elif scanAngle >= 30 and scanAngle <= 210:
        scanAngle -= 30
        x = dist * sin(scanAngle)
        y = dist * cos(scanAngle)
    elif scanAngle > 210:
        newScanAng = scanAngle - 210
        x = dist * sin(newScanAng) * -1
        y = dist * cos(newScanAng) * -1

    result = [x, y]
    return result

def findLaserXZ(angle):
    x = radius * sin(angle)
    z = radius * cos(angle)
    result = [x, z]
    return result

def findXYZ(laserXZ, scanXY, angle):
    if scanXY[0] < 0:
        # x direction is negative, i.e. go backwards
        print "X IS NEGATIVE"
        xOffset = scanXY[0] * cos(angle)
        zOffset = scanXY[0] * sin(angle) * -1

        print "xOffset: %f" % xOffset
        print "zOffset: %f" % zOffset

        x = laserXZ[0] + xOffset
        y = scanXY[1]
        z = laserXZ[1] + zOffset

        return [x, y, z]
    else:
        # x direction is positive, i.e. go forwards
        print "X IS POSITIVE"
        newAngle = 90 - angle
        xOffset = scanXY[0] * sin(newAngle)
        zOffset = scanXY[0] * cos(newAngle)

        print "xOffset: %f" % xOffset
        print "zOffset: %f" % zOffset

        x = laserXZ[0] + xOffset
        y = scanXY[1]
        z = laserXZ[1] - zOffset

        return [x, y, z]


def analyzeMsg():
    global currentSlice
    global sliceBuffer
    global sliceBuffMutex
    global currentRanges

    processSlice = False

    sliceBuffMutex.acquire()
    try:
        if len(sliceBuffer) > 0:
            currentSlice = sliceBuffer.pop(0)  # remove first element from buffer
            processSlice = True
    finally:
        sliceBuffMutex.release()

    if not processSlice:
        return


    currentSliceString = str(currentSlice)

    # Return only the ranges: [] part of the LaserScanAngle data
    findRanges(currentSliceString)

    # Return only the step angle part of the LaserScanAngle data
    findAngle(currentSliceString)

    # Find the number of data points in each scan by the commas
    numberScan = currentRanges.count(",", 0, len(currentRanges))

    start = 0

    for a in range(0, numberScan + 1): # numberScan + 1

        end = currentRanges.find(",", start)

        # Remove the space at the beginning
        if currentRanges[start:end].startswith(' '):
            start += 1

        # Find the last data set that does not have a ","
        if end == -1:
            end = len(currentRanges)

        # Check if the string is infinity or not availible
        if not currentRanges[start:end].startswith('inf') and not currentRanges[start:end].startswith('nan'):

            currentRangePoint = float(currentRanges[start:end])

            floatAngle = float(currentAngle)

            aFloat = float(a)
            numberScanFloat = float(numberScan)
            scanAngle = (aFloat / numberScanFloat) * scanRange

            print "---------------------------"
            scanXY = findScanXY(currentRangePoint, scanAngle)
            print "dist: %f" % currentRangePoint
            print "scanAngle: %f" % scanAngle
            print "scan x: %f" % scanXY[0]
            print "scan y: %f" % scanXY[1]

            laserXZ = findLaserXZ(floatAngle)
            print "Angle: %f" % floatAngle
            print "laserX: %f" % laserXZ[0]
            print "laserZ: %f" % laserXZ[1]

            XYZ = findXYZ(laserXZ, scanXY, floatAngle)

            x = XYZ[0]
            y = XYZ[1]
            z = XYZ[2]

            print "~~~~"
            print "x: %f" % x
            print "y: %f" % y
            print "z: %f" % z

            currPoint = [x, y, z]
            points.append(currPoint)

        # Make sure we grab the next range point
        start = end + 1

# Create a rate control object
rate = rospy.Rate(CYCLE_FREQUENCY)

print "Started PC Slice Node V2 at %d Hz." % CYCLE_FREQUENCY

while not rospy.is_shutdown():
    # Main loop

    analyzeMsg()

    # Instantiate a header
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "world"

    # Create a message of type sensor_msgs.PointCloud2
    pointCloud = point_cloud_message_creator.create_cloud_xyz32(header, points)

    cloudPublisher.publish(pointCloud)

    rate.sleep()


print "PCG done, waiting until ctrl+c is hit..."
rospy.spin()  # just to prevent this node from exiting