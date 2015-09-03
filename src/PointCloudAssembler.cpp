#include <ros/ros.h>

#include <PointCloudAssembler.hpp>

// #include <std_msgs/Float64MultiArray.h>
// #include <std_msgs/MultiArrayDimension.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <thread>
// #include <mutex>

#define QUEUE_SIZE 1
#define LATCHED false

namespace tiltingLIDARScanner {

PointCloudAssembler::PointCloudAssembler()
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
    return true;
}

bool PointCloudAssembler::stop()
{
    return true;
}

bool PointCloudAssembler::addSlice(const pcg_node::PointCloudSliceMsg & slice)
{
    return true;
}

void PointCloudAssembler::obtainSlice()
{
}

} // namespace tiltingLIDARScanner
