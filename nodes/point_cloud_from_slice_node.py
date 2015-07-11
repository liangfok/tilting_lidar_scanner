#!/usr/bin/env python

import rospy                        #for interacting with ROS topics and parameters
# import sys, getopt                  #for parameters and sys.exit()
from sensor_msgs.msg import PointCloud2, LaserScan
from pcg_node.msg import PointCloudSliceMsg
from std_msgs.msg import Header

import time
import math

import point_cloud_message_creator

import threading  # for mutex

import tf  # for tf broadcaster

# A buffer for holding point cloud slices
sliceBuffer = []

# Declare a mutex to prevent two threads from accessing the sliceBuffer
sliceBuffMutex = threading.Lock()

# The cycle frequency of the main loop
CYCLE_FREQUENCY = 1

# Define some constants
TILT_RADIUS = 0.08         # 0.08 meters
AXLE_HEIGHT = 1.1075       # 1.1075 meters

# A buffer for storing the points in the point cloud
points = []

def sliceCallback(msg):
    '''
    The callback method for point cloud slice messages.
    '''
    global sliceBuffer
    global sliceBuffMutex

    sliceBuffMutex.acquire()
    try:
        sliceBuffer.append(msg)  # Store the slice in the slice buffer
    finally:
        sliceBuffMutex.release()

def processSliceBuffer():
    '''
    Processes a slice from the slice buffer.
    Stores the results in global variable points.
    '''
    global sliceBuffer
    global sliceBuffMutex

    processSlice = False

    sliceBuffMutex.acquire()
    try:
        if len(sliceBuffer) > 0:
            currSlice = sliceBuffer.pop(0)  # remove first element from buffer
            processSlice = True
    finally:
        sliceBuffMutex.release()

    if not processSlice:
        return

    # print "PCFS: Processing slice: {0}".format(currSlice)

    # angleMin = currSlice.laserScan.angle_min
    # angleMax = currSlice.laserScan.angle_max
    angleMin = math.radians(-120)
    angleMax = math.radians(120)

    # angleInc = currSlice.laserScan.angle_increment
    angleInc = (angleMax - angleMin) / len(currSlice.laserScan.ranges)

    thetaT = math.radians(currSlice.tiltAngle)

    x0 = TILT_RADIUS * math.sin(thetaT)
    z0 = TILT_RADIUS * math.cos(thetaT)
    # print "PCFS: Values:\n"\
    #       "  - angleMin: {0}\n"\
    #       "  - angleMax: {1}\n"\
    #       "  - angleInc: {2}\n"\
    #       "  - tiltAngle: {3}\n".format(angleMin, angleMax, angleInc, tiltAngle)

    for ii in range(0, len(currSlice.laserScan.ranges)):
        thetaS = angleMin + ii * angleInc
        distS = currSlice.laserScan.ranges[ii]

        if not math.isinf(distS) and not math.isnan(distS):
            # Compute the coordinates of the point in the sensor's coordinate frame
            xS = distS * math.cos(thetaS)
            yS = distS * math.sin(thetaS)
            zS = 0

            # Convert from sensor coordinate frame to the base coordinate frame
            xB = xS * math.sin(thetaT) + x0
            yB = yS
            zB = xS * math.cos(thetaT) + z0

            print "PCFS: Values:\n"\
                  "  - angleMin / angleMax: {0} / {1} ({2} / {3})\n"\
                  "  - angleInc: {4} ({5})\n"\
                  "  - tiltAngle: {6} ({7})\n"\
                  "  - sensor coordinate frame:\n"\
                  "     - theta {8} ({9})\n"\
                  "     - dist: {10}\n"\
                  "     - position: ({11}, {12}, {13})\n"\
                  "  - base coordinate frame:\n"\
                  "     - position: ({14}, {15}, {16})\n".format(
                    angleMin, angleMax, math.degrees(angleMin), math.degrees(angleMax),
                    angleInc, math.degrees(angleInc),
                    thetaT, math.degrees(thetaT),
                    thetaS, math.degrees(thetaS), distS, xS, yS, zS,
                    xB, yB, zB)

            currPoint = [xB, yB, zB]
            points.append(currPoint)

if __name__ == "__main__":

    # initialize ROS node
    rospy.init_node('PC_Slice_Node', anonymous=True)

    # Instantiate a publisher for the sensor_msgs.PointCloud2 message
    cloudPublisher = rospy.Publisher("pointCloud", PointCloud2, queue_size = 0)

    # Instantiate a tf broadcaster for transforming world to base
    br = tf.TransformBroadcaster()

    # Instantiate the subscriber to point cloud slices
    scanSubscriber  = rospy.Subscriber("slice", PointCloudSliceMsg, sliceCallback)

    # Create a rate control object
    rate = rospy.Rate(CYCLE_FREQUENCY)

    print "PCFS: Started PC Slice Node V3 at %d Hz." % CYCLE_FREQUENCY

    # Main loop
    while not rospy.is_shutdown():

        # Process at most 100 slices per round
        ii = 0
        while len(sliceBuffer) > 0 and ii < 100:
            processSliceBuffer()
            ii = ii + 1

        # Broadcast world-to-base transform
        br.sendTransform((0, 0, AXLE_HEIGHT),
                     tf.transformations.quaternion_from_euler(0, math.radians(90), 0),  # rotate 90 degrees about Y axis
                     rospy.Time.now(),
                     "base",
                     "world")

        # Instantiate a header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "base"

        # Create a message of type sensor_msgs.PointCloud2
        pointCloud = point_cloud_message_creator.create_cloud_xyz32(header, points)

        # Publish the point cloud onto ROS topic pointCloud
        cloudPublisher.publish(pointCloud)

        rate.sleep()


    print "PCFS: Done, waiting until ctrl+c is hit..."
    rospy.spin()  # just to prevent this node from exiting