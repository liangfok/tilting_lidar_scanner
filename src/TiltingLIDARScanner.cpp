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
#define MCU_CMD_MSG_SIZE 2
#define MCU_CMD_START_BYTE 0x55
#define MCU_RESP_START_BYTE 0x66

// Commands that can be sent to the MCU
#define MCU_CMD_NONE 0
#define MCU_CMD_STEP 1
#define MCU_CMD_RECALIBRATE 2

// Valid commands that can be received by this node
#define CMD_STOP_SCAN 0
#define CMD_START_SCAN 1
#define CMD_RECALIBRATE 2

// The activation state of this node
#define STATE_DISABLED 0
#define STATE_ENABLED 1
#define STATE_RECALIBRATE 2

// The number of steps the motor must take to complete one 360 rotation
#define STEPS_PER_REV 4800

using LibSerial::SerialStreamBuf;

namespace tiltingLIDARScanner {

TiltingLIDARScanner::TiltingLIDARScanner() :
  serialPortName("/dev/ttyACM0"),
  laserScanTopic("/scan"),
  state(STATE_DISABLED)

{
    // Initialize the micro-controller command message
    mcuCmd[0] = MCU_CMD_START_BYTE;
    mcuCmd[1] = MCU_CMD_NONE;
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
    slicePublisher = nh.advertise<pcg_node::PointCloudSliceMsg>("slices", QUEUE_SIZE, LATCHED);

    // Initialize the ROS topic subscribers
    nh.getParam("laserScanTopic", laserScanTopic);
    ROS_INFO_STREAM("TiltingLIDARScanner::init: Subscribing to laser scan topic \"" << laserScanTopic << "\"");

    laserScanSubscriber = nh.subscribe(laserScanTopic, QUEUE_SIZE,
        & TiltingLIDARScanner::laserScanCallback, this);

    cmdSubscriber = nh.subscribe("cmd", QUEUE_SIZE,
        & TiltingLIDARScanner::cmdCallback, this);

    return true;
}

void TiltingLIDARScanner::sendMCUCmd(char command)
{
    mcuCmd[1] = command;
    serialPort.write(mcuCmd, MCU_CMD_MSG_SIZE);
    serialPort.flush();
}

bool TiltingLIDARScanner::rcvMCUMsg()
{
    bool result = false;
    char b0, b1, b2, b3, b4, b5;

    if (!serialPort.get(b0)) return false;
    if (b0 != MCU_RESP_START_BYTE) return false;
    if (!serialPort.get(b1)) return false;
    if (!serialPort.get(b2)) return false;
    if (!serialPort.get(b3)) return false;
    if (!serialPort.get(b4)) return false;
    if (!serialPort.get(b5)) return false;

    // Perform checksum
    char checksum;
    checksum = b0 ^ b1 ^ b2 ^ b3  ^ b4;
    if (checksum == b5)
    {
        stepPosition = (b1 << 24) + (b2 << 16) + (b3 << 8) + (b4 & 0xFF);
        currAngle = static_cast<double>(stepPosition) / STEPS_PER_REV * 360;

        // ROS_INFO_STREAM("TiltingLIDARScanner::rcvMCUMsg: Received current angle: " << currAngle << ", stepPosition: " << stepPosition << "\n"
        //     << "b0 = " << std::hex << "0x" << static_cast<int>(b0) << "\n"
        //     << "b1 = " << std::hex << "0x" << static_cast<int>(b1) << "\n"
        //     << "b2 = " << std::hex << "0x" << static_cast<int>(b2) << "\n"
        //     << "b3 = " << std::hex << "0x" << static_cast<int>(b3) << "\n"
        //     << "b4 = " << std::hex << "0x" << static_cast<int>(b4) << "\n"
        //     << "b5 = " << std::hex << "0x" << static_cast<int>(b5));
        result = true;
    }
    else
    {
        ROS_ERROR_STREAM("TiltingLIDARScanner::rcvMCUMsg: Bad checksum of 0x" << std::hex << checksum << ", expected 0x" << b5);
        result = false;
    }

    return result;
}

void TiltingLIDARScanner::obtainSlice()
{
    // Issue step command
    sendMCUCmd(MCU_CMD_STEP);
    ros::Time stepTime = ros::Time::now();

    // Wait for angle message
    while (ros::ok() && !rcvMCUMsg())
    {
        ros::spinOnce();
    }

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
            sliceMsg.tiltAngle = currAngle;
            slicePublisher.publish(sliceMsg);

            // TODO: Store slice in point cloud
        } else
            ros::spinOnce();
    }
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
            obtainSlice();
        else if (currState == STATE_DISABLED)
        {
            // Don't do anything
        }
        else if (currState == STATE_RECALIBRATE)
            sendMCUCmd(MCU_CMD_RECALIBRATE);

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