#include <ros/ros.h>

#include <PointCloud2MsgCreator.hpp>
#include <sensor_msgs/PointField.h>

namespace tiltingLIDARScanner {

PointCloud2MsgCreator::PointCloud2MsgCreator()
{
}

PointCloud2MsgCreator::~PointCloud2MsgCreator()
{
}

bool PointCloud2MsgCreator::createPointCloud2Msg(
    std::string & frameID,
    std::vector<Coordinate> points,
    sensor_msgs::PointCloud2 & pc2Msg)

{
    pc2Msg.header.stamp = ros::Time::now();
    pc2Msg.header.frame_id = frameID;
    pc2Msg.height = 1;
    pc2Msg.width = points.size();

    if (pc2Msg.fields.size() != 3)
    {
        sensor_msgs::PointField pfx, pfy, pfz;

        pfx.name = "x";
        pfx.offset = 0;
        pfx.datatype = sensor_msgs::PointField::FLOAT32;
        pfx.count = 1;

        pfy.name = "y";
        pfy.offset = 4;
        pfy.datatype = sensor_msgs::PointField::FLOAT32;
        pfy.count = 1;

        pfz.name = "z";
        pfz.offset = 8;
        pfz.datatype = sensor_msgs::PointField::FLOAT32;
        pfz.count = 1;

        pc2Msg.fields.clear();
        pc2Msg.fields.push_back(pfx);
        pc2Msg.fields.push_back(pfy);
        pc2Msg.fields.push_back(pfz);
    }

    pc2Msg.is_bigendian = false;
    pc2Msg.point_step = 12;
    pc2Msg.row_step = 12 * points.size();
    pc2Msg.data.resize(pc2Msg.row_step);
    pc2Msg.is_dense = false;

    for (int ii = 0; ii < points.size(); ii++)
    {
        int offset = ii * pc2Msg.point_step;

        unsigned char const * xPtr = reinterpret_cast<unsigned char const *>(&points[ii].x);
        pc2Msg.data[offset    ]  = xPtr[0];
        pc2Msg.data[offset + 1]  = xPtr[1];
        pc2Msg.data[offset + 2]  = xPtr[2];
        pc2Msg.data[offset + 3]  = xPtr[3];

        unsigned char const * yPtr = reinterpret_cast<unsigned char const *>(&points[ii].y);
        pc2Msg.data[offset + 4]  = yPtr[0];
        pc2Msg.data[offset + 5]  = yPtr[1];
        pc2Msg.data[offset + 6]  = yPtr[2];
        pc2Msg.data[offset + 7]  = yPtr[3];

        unsigned char const * zPtr = reinterpret_cast<unsigned char const *>(&points[ii].z);
        pc2Msg.data[offset + 8 ] = zPtr[0];
        pc2Msg.data[offset + 9 ] = zPtr[1];
        pc2Msg.data[offset + 10] = zPtr[2];
        pc2Msg.data[offset + 11] = zPtr[3];
    }

    return true;
}

} // namespace tiltingLIDARScanner
