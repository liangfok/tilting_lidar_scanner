#include <MCU.hpp>

#include "ros/ros.h"
// A command message to the micro-controller consists of two bytes:
// a start byte followed by a command byte.
#define MCU_CMD_MSG_SIZE 2
#define MCU_CMD_START_BYTE 0x55
#define MCU_RESP_START_BYTE 0x66

// Commands that can be sent to the MCU
#define MCU_CMD_NONE 0
#define MCU_CMD_STEP 1
#define MCU_CMD_RECALIBRATE 2

// The number of steps the motor must take to complete one 360 rotation
#define STEPS_PER_REV 4800

#define PRINT_ERROR(ss) ROS_ERROR_STREAM("[" << __FILE__ << ":" << __LINE__ << "] " \
    << "Error: " << ss);

using LibSerial::SerialStreamBuf;

namespace tiltingLIDARScanner {

MCU::MCU() :
    serialPortName("/dev/ttyACM0")
{
    // Initialize the micro-controller command message
    mcuCmd[0] = MCU_CMD_START_BYTE;
    mcuCmd[1] = MCU_CMD_NONE;
}

MCU::~MCU()
{
}

bool MCU::init()
{
    ros::NodeHandle nh;

    // Connect to the MCU via serial port
    nh.getParam("serialPort", serialPortName);
    ROS_INFO_STREAM("MCU::init: Connecting to serial port " << serialPortName);

    serialPort.Open(serialPortName);

    if (!serialPort.good())
    {
        PRINT_ERROR("Could not open serial port.");
        return false;
    }

    serialPort.SetBaudRate(SerialStreamBuf::BAUD_9600);
    if (!serialPort.good())
    {
        PRINT_ERROR("Could not set the baud rate.");
        return false;
    }

    // Set the number of data bits.
    serialPort.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
    if (!serialPort.good())
    {
        PRINT_ERROR("Could not set the character size.");
        return false;
    }

    // Disable parity.
    serialPort.SetParity(SerialStreamBuf::PARITY_NONE);
    if (!serialPort.good())
    {
        PRINT_ERROR("Could not disable the parity.");
        return false;
    }

    // Set the number of stop bits.
    serialPort.SetNumOfStopBits(1) ;
    if (!serialPort.good())
    {
        PRINT_ERROR("Could not set the number of stop bits.");
        return false;
    }

    // Turn off hardware flow control.
    serialPort.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
    if (!serialPort.good())
    {
        PRINT_ERROR("Could not use hardware flow control.");
        return false;
    }

    return true;
}

bool MCU::stop()
{
    serialPort.Close();
    return true;
}

bool MCU::recalibrate()
{
    sendMCUCmd(MCU_CMD_RECALIBRATE);
}

bool MCU::step(double & angle)
{
    // Tell stand to take a tilt step
    sendMCUCmd(MCU_CMD_STEP);

    // Wait for angle message from stand
    int cycleCount = 0;
    bool msgRcvd = false;
    ros::Rate loopRate(100);

    while (ros::ok() && !(msgRcvd = rcvMCUMsg(angle)) && cycleCount++ < 100)
    {
        ros::spinOnce();
        loopRate.sleep();
    }

    return msgRcvd;
}

void MCU::sendMCUCmd(char command)
{
    mcuCmd[1] = command;
    serialPort.write(mcuCmd, MCU_CMD_MSG_SIZE);
    serialPort.flush();
}

bool MCU::rcvMCUMsg(double & angle)
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
        int stepPosition = (b1 << 24) + (b2 << 16) + (b3 << 8) + (b4 & 0xFF);
        angle = static_cast<double>(stepPosition) / STEPS_PER_REV * 360;

        // ROS_INFO_STREAM("MCU::rcvMCUMsg: Received current angle: " << currAngle << ", stepPosition: " << stepPosition << "\n"
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
        ROS_ERROR_STREAM("MCU::rcvMCUMsg: Bad checksum of 0x" << std::hex << checksum << ", expected 0x" << b5);
        result = false;
    }

    return result;
}


} // namespace tiltingLIDARScanner
