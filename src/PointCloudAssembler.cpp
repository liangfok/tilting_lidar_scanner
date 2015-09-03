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
#define AXLE_HEIGHT 1.1075

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
            currPoint.x = xB;
            currPoint.y = yB;
            currPoint.z = zB;

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
    if (pc2MsgCreator.createPointCloud2Msg(pointBuff, pc2Msg))
    {
        pcPublisher.publish(pc2Msg);
    }
}
} // namespace tiltingLIDARScanner
