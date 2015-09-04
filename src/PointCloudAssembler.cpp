#include <ros/ros.h>

#include <PointCloudAssembler.hpp>

// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/MultiArrayDimension.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <math.h> // for sin() and cos()

#define QUEUE_SIZE 1
#define LATCHED false

#define TILT_RADIUS 0.08

namespace tiltingLIDARScanner {

PointCloudAssembler::PointCloudAssembler() :
    done(false)
{
}

PointCloudAssembler::~PointCloudAssembler()
{
}

bool PointCloudAssembler::init()
{
    // Initialize the ROS topic publishers
    pcPublisher = nh.advertise<sensor_msgs::PointCloud2>("pointCloud", QUEUE_SIZE, LATCHED);

    // Get the name of the point cloud's frame
    if (!nh.getParam("frameID", frameID))
        frameID = "unknown";

    // Get the transforms from the world frame to the point cloud's frame
    if (!nh.getParam("xOffset", xOffset))
        xOffset = 0;

    if (!nh.getParam("yOffset", yOffset))
        yOffset = 0;

    if (!nh.getParam("zOffset", zOffset))
        zOffset = 0;

    if (!nh.getParam("roll", roll))
        roll = 0;

    if (!nh.getParam("pitch", pitch))
        pitch = 0;

    if (!nh.getParam("yOffset", yaw))
        yaw = 0;

    return true;
}

bool PointCloudAssembler::start()
{
    // Create a thread that processes the point cloud slices
    thread = std::thread(&PointCloudAssembler::processSlices, this);
    return true;
}

bool PointCloudAssembler::stop()
{
    done = true;
    thread.join();
    return true;
}

bool PointCloudAssembler::addSlice(const pcg_node::PointCloudSliceMsg & slice)
{
    sliceMutex.lock();
    sliceBuff.push_back(slice);
    sliceMutex.unlock();
    return true;
}

void PointCloudAssembler::processSlices()
{
    pcg_node::PointCloudSliceMsg currSlice;

    ros::Rate loopRate(1);
    while (!done && ros::ok())
    {
        int numSlices = 0;

        sliceMutex.lock();
        numSlices = sliceBuff.size();
        sliceMutex.unlock();

        // ROS_INFO_STREAM("PointCloudAssembler::processSlices: Processing "
        //     << numSlices << " slices...");

        while (numSlices-- > 0)
        {
            // Obtain and remove the oldest slice in the buffer
            sliceMutex.lock();
            currSlice = sliceBuff[0];
            sliceBuff.erase(sliceBuff.begin());
            sliceMutex.unlock();

            // Process slice
            processSlice(currSlice);
        }

        // Generate and publish the latest point cloud
        createAndPublishPointCloud();

        loopRate.sleep();
    }
}

void PointCloudAssembler::processSlice(const pcg_node::PointCloudSliceMsg & currSlice)
{
    // Obtain the min and max angle from the slice
    double angleMin = currSlice.laserScan.angle_min;
    double angleMax = currSlice.laserScan.angle_max;

    // Obtain the slice's increment angle
    double angleInc = currSlice.laserScan.angle_increment;
    double theoreticalAngleInc = (angleMax - angleMin) / currSlice.laserScan.ranges.size();

    // Obtain the tilt angle
    double thetaT = currSlice.tiltAngle * M_PI / 180.0;

    double x0 = TILT_RADIUS * sin(thetaT);
    double z0 = TILT_RADIUS * cos(thetaT);

    int numSamples = currSlice.laserScan.ranges.size();

    for (int ii = 0; ii < numSamples; ii++)
    {
        double thetaS = angleMin + ii * angleInc;
        double distS = currSlice.laserScan.ranges[ii];

        if (!isinf(distS) && !isnan(distS))
        {
            // Compute the coordinates of the point in the sensor's coordinate frame
            double xS = distS * cos(thetaS);
            double yS = distS * sin(thetaS);
            double zS = 0;

            // Convert from sensor coordinate frame to the base coordinate frame
            double xB = xS * cos(-thetaT) + x0;
            double yB = yS;
            double zB = xS * sin(-thetaT) + z0;

            // ROS_INFO_STREAM("PointCloudAssembler::processSlice: Values:\n"
            //     << "  - angleMin / angleMax: " << angleMin << " / " << angleMax << " (" << toDeg(angleMin) << " / " << toDeg(angleMax) << ")\n"
            //     << "  - angleInc: " << angleInc << " (" << toDeg(angleInc) << ")\n"
            //     << "  - theoretical angleInc: " << theoreticalAngleInc << " (" << toDeg(theoreticalAngleInc) << ")\n"
            //     << "  - number of samples: " << numSamples << "\n"
            //     << "  - tiltAngle: " << thetaT << " (" << toDeg(thetaT) << ")\n"
            //     << "  - sensor coordinate frame:\n"
            //     << "     - theta " << thetaS << " (" << toDeg(thetaS) << ")\n"
            //     << "     - dist: " << distS << "\n"
            //     << "     - position: (" << xS << ", " << yS << ", " << zS << ")\n"
            //     << "  - base coordinate frame:\n"\
            //     << "     - position: (" << xB << ", " << yB << ", " << zB << ")");

            Coordinate currPoint;
            currPoint.x = (float)xB;
            currPoint.y = (float)yB;
            currPoint.z = (float)zB;

            pointBuff.push_back(currPoint);
        }
    }
}

double PointCloudAssembler::toDeg(double rad)
{
    return rad / M_PI * 180.0;
}

void PointCloudAssembler::createAndPublishPointCloud()
{
    sensor_msgs::PointCloud2 pc2Msg;
    if (pc2MsgCreator.createPointCloud2Msg(frameID, pointBuff, pc2Msg))
        pcPublisher.publish(pc2Msg);

    // Publish frame offset information
    geometry_msgs::TransformStamped transMsg;

    transMsg.header.frame_id = "world";
    transMsg.child_frame_id = frameID;

    transMsg.header.stamp = ros::Time::now();
    transMsg.transform.translation.x = xOffset;
    transMsg.transform.translation.y = yOffset;
    transMsg.transform.translation.z = zOffset;

    tf::Quaternion quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

    transMsg.transform.rotation.x = quat.x();
    transMsg.transform.rotation.y = quat.y();
    transMsg.transform.rotation.z = quat.z();
    transMsg.transform.rotation.w = quat.w();

    tfBroadcaster.sendTransform(transMsg);

}
} // namespace tiltingLIDARScanner
