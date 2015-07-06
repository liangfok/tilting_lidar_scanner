#!/usr/bin/env python

import rospy                        #for interacting with ROS topics and parameters
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

import math
import random
import struct
import ctypes

import point_cloud_message_creator

# The frequency of publishing the PointCloud2 message
PUBLISH_FREQUENCY = 10

def genRandom(min, max):
    delta = abs(max - min)
    rand = random.random()
    return min + (rand * delta)

# Generates a random point on a sphere
# See: http://stackoverflow.com/questions/5408276/python-uniform-spherical-distribution
def genRandomPointOnSphere(radius):
    phi = genRandom(0, 2 * math.pi)
    costheta = genRandom(-1, 1)
    u = genRandom(0, 1)

    theta = math.acos(costheta)
    r = radius * (u ** (1 / 3.0))

    # theta is angle relative to Z axis
    # phi is angle relative to X axis

    x = r * math.sin(theta) * math.cos(phi)
    y = r * math.sin(theta) * math.sin(phi)
    z = r * math.cos(theta)

    result = []
    result.append(x)
    result.append(y)
    result.append(z)

    return result

if __name__ == "__main__":

    # Initialize the ROS node
    rospy.init_node('point_cloud_example_sphere', anonymous=True)

    # Create a rate control object
    rate = rospy.Rate(PUBLISH_FREQUENCY)

    # Instantiate a publisher for the sensor_msgs.PointCloud2 message
    cloudPublisher = rospy.Publisher("pointCloud", PointCloud2, queue_size = 0)

    while not rospy.is_shutdown():

        # Instantiate a 'points' array and fill it up with random points on a sphere
        points = []

        for x in range(0, 10000):
            currPoint = genRandomPointOnSphere(1.0)
            points.append(currPoint)

        # Instantiate a header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "world"

        # Create a message of type sensor_msgs.PointCloud2
        pointCloud = point_cloud_message_creator.create_cloud_xyz32(header, points)

        cloudPublisher.publish(pointCloud)
        rate.sleep()

    print "Sphere point cloud example publisher done. Waiting until ctrl+c is hit..."
    rospy.spin()  # just to prevent this node from exiting