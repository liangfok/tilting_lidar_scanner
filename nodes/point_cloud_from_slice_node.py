#!/usr/bin/env python

import rospy                        #for interacting with ROS topics and parameters
import sys, getopt                  #for parameters and sys.exit()
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import PointCloud2, LaserScan
from pcg_node.msg import LaserScanAngle

import re                           #for findall()
import string                       #for split()
import time
import math

# Change this to change the speed of this program:
hertz = 100

sliceBuffer = []
counter = 0
analyzeCounter = 0

currentSlice = 0
currentSliceString = ""
currentRanges = ""
currentRangePoint = 0
currentAngle = 0

# Data for calculating the actual distance
radius = 0.08         # 0.08 meters (80 mm)
axleHeight = 0.1075   # 0.1075 meters (107.5mm)


# initialize ROS node
rospy.init_node('PC_Slice_Node', anonymous=True)

# Declare some ROS topic subscriber callback function
def sliceCallback(msg):
    global sliceBuffer
    global counter

    # Store the slice in the slice buffer
    sliceBuffer.append(msg)
    counter += 1
    #print "Message received #%d" % counter
    #print msg

# Instantiate the subscribers
scanSubscriber  = rospy.Subscriber("slice", LaserScanAngle, sliceCallback)

def findTrueDist(origDist):
    global radius

    # The math
    d2 = math.pow(origDist, 2)
    r2 = math.pow(radius, 2)
    result = math.sqrt(d2 + r2)

    return result

def findTrueAngle(origAngle, currRangePoint):
    global currentAngle
    global radius

    frac = (currRangePoint / radius)
    invTan = math.degrees(math.atan(frac))

    floatAngle = float(currentAngle)

    result = invTan + floatAngle

    return result


def findRanges(origData):
    global currentRanges
    beg = "["
    end = "]"

    ran = origData.find(beg)
    ran2 = origData.find(end)

    #print "FOUND RANGE: -----------------------"
    #print origData[ran + 1:ran2]

    currentRanges = origData[ran + 1:ran2]

def findAngle(origData):
    global currentAngle

    beg = "stepAngle:"

    ang = origData.find(beg)
    ang2 = len(origData)

    # Cut the string off so that we only get the angle and not stepAngle
    currentAngle = origData[ang + 11:ang2]

def findX(dist, angle, scanAngle):
    base = 0.0
    # Find the base line
    if angle < 90:
        rad = math.radians(angle)
        sinVal = math.sin(rad)
        base = (dist * sinVal)
    elif angle == 90:
        base = dist
    elif angle > 90:
        newAng = angle - 90
        rad = math.radians(newAng)
        cosVal = math.cos(rad)
        base = (dist * cosVal)

    # Find the x component from there
    newScanAng = 0.0

    if scanAngle == 90:
        print "x: %f" % dist
        return dist
    elif scanAngle > 90:
        newScanAng = 180 - scanAngle
    elif scanAngle < 90:
        newScanAng = scanAngle

    rad = math.radians(newScanAng)
    sinVal = math.sin(rad)
    result = base * sinVal
    print "x: %f" % result
    return result

def findY(dist, angle, scanAngle):
    base = 0.0
    # Find the base line
    if angle < 90:
        rad = math.radians(angle)
        sinVal = math.sin(rad)
        base = (dist * sinVal)
    elif angle == 90:
        base = dist
    elif angle > 90:
        newAng = angle - 90
        rad = math.radians(newAng)
        cosVal = math.cos(rad)
        base = (dist * cosVal)

    # Find the x component from there
    newScanAng = 0.0

    if scanAngle == 90:
        print "y: %f" % dist
        return dist
    elif scanAngle > 90:
        newScanAng = 180 - scanAngle
    elif scanAngle < 90:
        newScanAng = scanAngle

    rad = math.radians(newScanAng)
    cosVal = math.cos(rad)
    result = base * cosVal
    print "y: %f" % result
    return result

def findZ(dist, angle, scanAngle):
    global axleHeight

    if angle < 90:
        triangleAng = 90 - angle
        rad = math.radians(triangleAng)
        sinVal = math.sin(rad)
        result = axleHeight
        #print "z: %f" % result
        result += (dist * sinVal)
        print "z: %f" % result
        return result
    elif angle == 90:
        result = axleHeight
        print "z: %f" % result
        return result
    elif angle > 90:
        newAng = angle - 90
        #print "newAng: %f" % newAng
        rad = math.radians(newAng)
        #print "rad: %f" % rad
        sinVal = math.sin(rad)
        #print "sinVal: %f" % sinVal
        result = axleHeight
        result -= (dist * sinVal)
        print "z: %f" % result
        return result



def analyzeMsg():
    global currentSlice
    global sliceBuffer
    global analyzeCounter
    global currentRanges


    if len(sliceBuffer) > analyzeCounter:
    #if len(sliceBuffer) == 1:
        currentSlice = sliceBuffer[analyzeCounter]
        currentSliceString = str(currentSlice)

        # Return only the ranges: [] part of the LaserScanAngle data
        findRanges(currentSliceString)        
        #print currentRanges

        # Return only the step angle part of the LaserScanAngle data
        findAngle(currentSliceString)
        print currentAngle


        # Find the number of data points in each scan by the commas
        numberScan = currentRanges.count(",", 0, len(currentRanges))
        print numberScan

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
                #print "Finding from %d to %d" % (start, end)
                #print currentRanges[start:end]
                currentRangePoint = float(currentRanges[start:end])
                #print currentRangePoint

                # Here's where we do the math for converting the points
                # from a distorted perspective to a normal one            

                print "-------------------------------"

                #print "Laser distance: %.12f" % currentRangePoint
                trueDist = findTrueDist(currentRangePoint)
                print "Actual distance: %.12f" % trueDist

                floatAngle = float(currentAngle)
                #print "Laser Angle: %f" % floatAngle
                trueAngle = findTrueAngle(floatAngle, currentRangePoint)
                print "Actual Angle is %.12f" % trueAngle

                aFloat = float(a)
                numberScanFloat = float(numberScan)
                scanAngle = (aFloat / numberScanFloat) * 180
                print "Scan angle is: %.12f" % scanAngle 

                # Set these variables as floats
                x = 0.0
                y = 0.0
                z = 0.0

                x = findX(trueDist, trueAngle, scanAngle)
                y = findY(trueDist, trueAngle, scanAngle)
                z = findZ(trueDist, trueAngle, scanAngle)


            # Make sure we grab the next range point
            start = end + 1
            

    
        analyzeCounter += 1



# Create a rate control object
rate = rospy.Rate(hertz)

print "Started PC Slice Node at %d hertz." % hertz

while not rospy.is_shutdown():
    # Main loop
    
    analyzeMsg()

    rate.sleep()


print "PCG done, waiting until ctrl+c is hit..."
rospy.spin()  # just to prevent this node from exiting