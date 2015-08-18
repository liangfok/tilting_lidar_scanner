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

// A command message to the micro-controller consists of two bytes:
// a start byte followed by a command byte.
#define CMD_MSG_SIZE 2
#define CMD_START_BYTE 0x55

// Valid commands
#define CMD_STOP_SCAN 0
#define CMD_START_SCAN 1
#define CMD_RECALIBRATE 2

// The activation state of this node
#define STATE_DISABLED 0
#define STATE_ENABLED 1
#define STATE_RECALIBRATE 2


using LibSerial::SerialStreamBuf;

namespace tiltingLIDARScanner {

TiltingLIDARScanner::TiltingLIDARScanner() :
  serialPortName("/dev/ttyACM0"),
  laserScanTopic("/scan"),
  state(STATE_DISABLED)

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

    laserScanSubscriber = nh.subscribe(laserScanTopic, QUEUE_SIZE,
        & TiltingLIDARScanner::laserScanCallback, this);

    cmdSubscriber = nh.subscribe("cmd", QUEUE_SIZE,
        & TiltingLIDARScanner::cmdCallback, this);

    return true;
}

void TiltingLIDARScanner::sendRecalibrateCmd()
{
    cmd[0] = CMD_START_BYTE;
    cmd[1] = CMD_RECALIBRATE;
    serialPort.write(cmd, CMD_MSG_SIZE);
    serialPort.flush();
}

bool TiltingLIDARScanner::start()
{
    int currState;

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        stateMutex.lock();
        currState = state;
        if (state == STATE_RECALIBRATE)
            state = STATE_DISABLED;
        stateMutex.unlock();

        if (currState == STATE_ENABLED)
        {
            // 
        }
        else if (currState == STATE_DISABLED)
        {
            // Don't do anything
        }
        else if (currState == STATE_RECALIBRATE)
            sendRecalibrateCmd();

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