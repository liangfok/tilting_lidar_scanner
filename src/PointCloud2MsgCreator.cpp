#include <ros/ros.h>

#include <PointCloud2MsgCreator.hpp>

namespace tiltingLIDARScanner {

PointCloud2MsgCreator::PointCloud2MsgCreator()
{
}

PointCloud2MsgCreator::~PointCloud2MsgCreator()
{
}

bool PointCloud2MsgCreator::createPointCloud2Msg(
    std::vector<Coordinate> points,
    sensor_msgs::PointCloud2 & pc2Msg)

{
    pc2Msg.header.stamp = ros::Time::now();
    pc2Msg.header.frame_id = "tilting_laser_scanner";
    

    return true;
}

} // namespace tiltingLIDARScanner
