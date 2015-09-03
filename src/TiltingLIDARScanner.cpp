#include <ros/ros.h>

#include <TiltingLIDARScanner.hpp>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

// #include <thread>
// #include <mutex>

#define QUEUE_SIZE 1
#define LATCHED false

// Valid commands that can be received by this node
#define CMD_STOP_SCAN 0
#define CMD_START_SCAN 1
#define CMD_RECALIBRATE 2

// The activation state of this node
#define STATE_DISABLED 0
#define STATE_ENABLED 1
#define STATE_RECALIBRATE 2

namespace tiltingLIDARScanner {

TiltingLIDARScanner::TiltingLIDARScanner() :
  laserScanTopic("/scan"),
  state(STATE_DISABLED)

{
}

TiltingLIDARScanner::~TiltingLIDARScanner()
{
}

bool TiltingLIDARScanner::init()
{
    // Initialize the ROS topic publishers
    slicePublisher = nh.advertise<pcg_node::PointCloudSliceMsg>("slices", QUEUE_SIZE, LATCHED);

    // Initialize the ROS topic subscribers
    nh.getParam("laserScanTopic", laserScanTopic);
    ROS_INFO_STREAM("TiltingLIDARScanner::init: Subscribing to laser scan topic \"" << laserScanTopic << "\"");

    laserScanSubscriber = nh.subscribe(laserScanTopic, QUEUE_SIZE,
        & TiltingLIDARScanner::laserScanCallback, this);

    cmdSubscriber = nh.subscribe("cmd", QUEUE_SIZE,
        & TiltingLIDARScanner::cmdCallback, this);

    if (pc.init())
        return mcu.init();
    else
        return false;
}

bool TiltingLIDARScanner::obtainSlice()
{
    // Issue step command
    double angle;

    if (!mcu.step(angle)) return false;

    ros::Time stepTime = ros::Time::now();

    // Wait for laser scan
    bool rcvdLaserScan = false;
    while (!rcvdLaserScan)
    {
        scanMutex.lock();
        if ((laserScan.header.stamp - stepTime).toSec() > 0)
        {
            sliceMsg.laserScan = laserScan;
            rcvdLaserScan = true;
        }
        scanMutex.unlock();

        if (rcvdLaserScan)
        {
            sliceMsg.tiltAngle = angle;
            slicePublisher.publish(sliceMsg);
            pc.addSlice(sliceMsg); // Store slice in point cloud
        } else
            ros::spinOnce();
    }
}

bool TiltingLIDARScanner::start()
{
    int currState;

    pc.start(); // start the point cloud generator

    ros::Rate loopRate(100);
    while (ros::ok())
    {
        stateMutex.lock();
        currState = state;
        if (state == STATE_RECALIBRATE)
            state = STATE_DISABLED;
        stateMutex.unlock();

        if (currState == STATE_ENABLED)
            obtainSlice();
        else if (currState == STATE_DISABLED)
        {
            // Don't do anything
        }
        else if (currState == STATE_RECALIBRATE)
            mcu.recalibrate();

        ros::spinOnce();
        loopRate.sleep();
    }

    return true;
}

bool TiltingLIDARScanner::stop()
{
    mcu.stop();
    return true;
}

void TiltingLIDARScanner::laserScanCallback(const sensor_msgs::LaserScan & scan)
{
    scanMutex.lock();
    this->laserScan = scan;
    // ROS_INFO_STREAM("TiltingLIDARScanner::laserScanCallback: Received laser scan at time " << scan.header.stamp);
    scanMutex.unlock();
}

void TiltingLIDARScanner::cmdCallback(const std_msgs::Int32 & cmd)
{
    stateMutex.lock();
    if (cmd.data == CMD_START_SCAN)
        state = STATE_ENABLED;
    else if (cmd.data == CMD_STOP_SCAN)
        state = STATE_DISABLED;
    else if (cmd.data == CMD_RECALIBRATE)
        state = STATE_RECALIBRATE;
    stateMutex.unlock();
}

} // namespace tiltingLIDARScanner

int main(int argc, char** argv)
{
    ROS_INFO_STREAM("TiltingLIDARScanner: main: Starting!");

    ros::init(argc, argv, "TiltingLIDARScanner");

    tiltingLIDARScanner::TiltingLIDARScanner scanner;
    if (scanner.init())
    {
        if(scanner.start())
        {
            scanner.stop();
            return 0;
        }
        else
        {
            scanner.stop();
            return -1;
        }
    }
}