#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <thread>
#include <mutex>

int main(int argc, char** argv)
{
    ROS_INFO_STREAM("TiltingLIDARScanner: main: Starting!");

    ros::init(argc, argv, "TiltingLIDARScanner");

    ros::NodeHandle nh;

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ROS_INFO_STREAM("Hello World!");
        ros::spinOnce();
        loop_rate.sleep();
    }
}