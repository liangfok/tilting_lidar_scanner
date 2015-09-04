#!/usr/bin/env python

'''
Subscribes to /slice and publishes to /scan.
Converts from PointCloudSliceMsg to LaserScan messages and tf messages.
'''

import rospy                        #for interacting with ROS topics and parameters
# import sys, getopt                  #for parameters and sys.exit()
from sensor_msgs.msg import PointCloud2, LaserScan
from tilting_lidar_scanner.msg import PointCloudSliceMsg
from std_msgs.msg import Header
from laser_assembler.srv import AssembleScans2

import time
import math

import point_cloud_message_creator

import threading  # for mutex

import tf  # for tf broadcaster

# A buffer for holding point cloud slices
sliceBuffer = []

# Declare a mutex to prevent two threads from accessing the sliceBuffer
inputBuffMutex = threading.Lock()

# The cycle frequency of the main loop
CYCLE_FREQUENCY = 1

# Define some constants
TILT_RADIUS = 0.08         # 0.08 meters
AXLE_HEIGHT = 1.1075       # 1.1075 meters

# Buffers
inputBuffer = []     # for holding tilting_lidar_scanner/PointCloudSliceMsg messages
# sliceBuffer = []     # for holding the point cloud slices
transformBuffer = dict() # for holding transforms

class Transform:
    '''
    Contains the transform from the sensor's "base" frame to a point cloud slice's coordinate frame.
    '''

    def __init__(self, translation, rotation, childFrame):
        '''
        The constructor.

        Keyword arguments:

          - translation: The translation from the "base" frame to the child frame. An (x, y, z) tuple.
          - rotation: The rotation from the "base" frame to the child frame. An (x, y, z , w) tuple.
          - childFrame: The name of the child frame.
        '''

        self.translation = translation
        self.rotation = rotation
        self.childFrame = childFrame

    def __str__(self):
        return "Transform from base frame:\n"\
               "  - childFrame: {0}\n"\
               "  - translation: {1}\n"\
               "  - rotation: {2}".format(self.childFrame, self.translation, self.rotation)

    def __repr__(self):
        return self.__str__()

def sliceCallback(msg):
    '''
    The callback method for point cloud slice messages.
    '''
    global inputBuffer
    global inputBuffMutex

    # rospy.loginfo("ALTA: sliceCallback: Received slice!")

    inputBuffMutex.acquire()
    try:
        inputBuffer.append(msg)  # Store the slice in the slice buffer
    finally:
        inputBuffMutex.release()

def processSliceBuffer():
    '''
    Processes a slice from the input buffer.
    Stores the results in the global sliceBuffer and transformBuffer.
    '''
    global inputBuffer
    global inputBuffMutex

    processSlice = False

    inputBuffMutex.acquire()
    try:
        if len(inputBuffer) > 0:
            currSlice = inputBuffer.pop(0)  # remove first element from buffer
            processSlice = True
    finally:
        inputBuffMutex.release()

    if not processSlice:
        return

    # tf.transformations.quaternion_from_euler(0, math.radians(90), 0),  # rotate 90 degrees about Y axis

    thetaT = math.radians(currSlice.tiltAngle)

    # If the child coordinate frame is not already in the transform buffer, add it to the transform buffer.
    childFrame =  "base_to_sensor_{0}".format(thetaT)
    if not childFrame in transformBuffer:

        # Compute the translation
        translation = (TILT_RADIUS * math.sin(thetaT), 0, TILT_RADIUS * math.cos(thetaT))

        # Compute the rotation. The sensor is tilting around the base coordinate frame's Y axis
        rotation = tf.transformations.quaternion_from_euler(0, thetaT, 0)

        # Instantiate a new transform
        transform = Transform(translation, rotation, childFrame)

        # Add transform to transform buffer
        # rospy.loginfo("ALTA: processSliceBuffer: Adding transform for frame {0}.".format(childFrame))
        transformBuffer[childFrame] = transform

        # Transmit the transform
        br.sendTransform(transform.translation,
                         transform.rotation,
                         rospy.Time.now(),
                         transform.childFrame,
                         "base")

    # Update the laser scan message and publish the slice
    currSlice.laserScan.header.stamp = rospy.Time.now()
    currSlice.laserScan.header.frame_id = childFrame




    # rospy.loginfo("ALTA: processSliceBuffer: Publishing laser scan!")
    scanPublisher.publish(currSlice.laserScan)

if __name__ == "__main__":

    # initialize ROS node
    rospy.init_node('PC_Slice_Node', anonymous=True)

    # Instantiate a publisher for the sensor_msgs.PointCloud2 message
    scanPublisher = rospy.Publisher("scan", LaserScan, queue_size = 0)

    # Instantiate a publisher for the sensor_msgs.PointCloud2 message
    cloudPublisher = rospy.Publisher("pointCloud", PointCloud2, queue_size = 0)

    # Instantiate a service for assembling scans
    assembleScansService = rospy.ServiceProxy('/assemble_scans2', AssembleScans2)

    beginTime = rospy.Time.now()

    # Instantiate a tf broadcaster for transforming world to base
    br = tf.TransformBroadcaster()

    # Instantiate the subscriber to point cloud slices
    scanSubscriber = rospy.Subscriber("slice", PointCloudSliceMsg, sliceCallback)

    # Create a rate control object
    rate = rospy.Rate(CYCLE_FREQUENCY)

    rospy.loginfo("ATLA: Started at %d Hz." % CYCLE_FREQUENCY)

    # Main loop
    while not rospy.is_shutdown():

        # rospy.loginfo("ATLA: length of input buffer: {0}".format(len(inputBuffer)))

        # Process at most 100 slices per round
        ii = 0
        while len(inputBuffer) > 0 and ii < 100:
            processSliceBuffer()
            ii = ii + 1

        try:
            response = assembleScansService(beginTime, rospy.Time.now())
            if len(response.cloud.data) != 0:
                cloudPublisher.publish(response.cloud)
            else:
                rospy.loginfo("ATLA: No point cloud returned from service call.")
        except rospy.ServiceException, e:
            rospy.logwarn("ATLA: Service call failed: %s" % e)

        # Broadcast transforms from world to child frames
        for transform in transformBuffer.itervalues():
            # rospy.loginfo("ALTA: Sending the following transform: {0}".format(transform))
            br.sendTransform(transform.translation,
                             transform.rotation,
                             rospy.Time.now(),
                             transform.childFrame,
                             "base")

        # Broadcast world-to-base transform
        br.sendTransform((0, 0, AXLE_HEIGHT),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "base",
                     "world")

        # Instantiate a header
        # header = Header()
        # header.stamp = rospy.Time.now()
        # header.frame_id = "base"

        # Create a message of type sensor_msgs.PointCloud2
        # pointCloud = point_cloud_message_creator.create_cloud_xyz32(header, points)

        # Publish the point cloud onto ROS topic pointCloud
        # cloudPublisher.publish(pointCloud)

        rate.sleep()


    rospy.loginfo("ATLA: Done, waiting until ctrl+c is hit...")
    rospy.spin()  # just to prevent this node from exiting