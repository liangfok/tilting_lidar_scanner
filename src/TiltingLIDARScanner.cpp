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

#define CMD_STOP_SCAN 0
#define CMD_START_SCAN 1

using LibSerial::SerialStreamBuf;

namespace tiltingLIDARScanner {

TiltingLIDARScanner::TiltingLIDARScanner() :
  serialPortName("/dev/ttyACM0"),
  laserScanTopic("/scan"),
  cmd(CMD_STOP_SCAN)
{
}

TiltingLIDARScanner::~TiltingLIDARScanner()
{
}

bool TiltingLIDARScanner::init()
{
    // Connect to the Arduino via serial port
    nh.getParam("serialPort", serialPortName);
    ROS_INFO_STREAM("TiltingLIDARScanner::init: Connecting to serial port " << serialPortName);

    serialPort.Open(serialPortName);

    if (!serialPort.good())
    {
        ROS_ERROR_STREAM("[" << __FILE__ << ":" << __LINE__ << "] "
            << "Error: Could not open serial port.");
        return false;
    }

    serialPort.SetBaudRate(SerialStreamBuf::BAUD_9600);
    if (!serialPort.good())
    {
        ROS_ERROR_STREAM("Error: Could not set the baud rate.");
        return false;
    }

    // Set the number of data bits.
    serialPort.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    if (!serialPort.good())
    {
        ROS_ERROR_STREAM("Error: Could not set the character size.");
        return false;
    }

    // Disable parity.
    serialPort.SetParity(SerialStreamBuf::PARITY_NONE);
    if (!serialPort.good())
    {
        ROS_ERROR_STREAM("Error: Could not disable the parity.");
        return false;
    }

    // Set the number of stop bits.
    serialPort.SetNumOfStopBits(1) ;
    if (!serialPort.good())
    {
        ROS_ERROR_STREAM("Error: Could not set the number of stop bits.");
        return false;
    }

    // Turn off hardware flow control.
    serialPort.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    if (!serialPort.good())
    {
        ROS_ERROR_STREAM("Error: Could not use hardware flow control.");
        return false;
    }

    // Initialize the ROS topic publishers
    slicePublisher = nh.advertise<sensor_msgs::LaserScan>("slices", QUEUE_SIZE, LATCHED);
    pcPublisher = nh.advertise<sensor_msgs::PointCloud2>("pointCloud", QUEUE_SIZE, LATCHED);

    // Initialize the ROS topic subscribers
    nh.getParam("laserScanTopic", laserScanTopic);
    ROS_INFO_STREAM("TiltingLIDARScanner::init: Subscribing to laser scan topic \"" << laserScanTopic << "\"");
    laserScanSubscriber.subscribe(laserScanTopic, boost::bind( & TiltingLIDARScanner::laserScanCallback, this, _1));

    cmdSubscriber.subscriber("cmd", boost::bind( & TiltingLIDARScanner::cmdCallback, this, _1));
    return true;
}

bool TiltingLIDARScanner::start()
{
    int localCmd;

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        cmdMutex.lock();
        localCmd = cmd;
        cmdMutex.unlock();

        if (localCmd == CMD_START_SCAN)
        {

        }
        else
        {

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return true;
}

bool TiltingLIDARScanner::stop()
{
    serialPort.Close();
    return true;
}

void TiltingLIDARScanner::laserScanCallback(const boost::shared_ptr<sensor_msgs::LaserScan const> & scan)
{
    scanMutex.lock();
    this->laserScan = scan;
    scanMutex.unlock();
}

void TiltingLIDARScanner::cmdCallback(const boost::shared_ptr<std_msgs::Int32 const> & cmd)
{
    cmdMutex.lock();
    this->cmd = cmd;
    cmdMutex.unlock();
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