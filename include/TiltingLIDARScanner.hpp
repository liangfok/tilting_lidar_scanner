/*
 * Copyright (C) 2015 The University of Texas at Austin.
 * All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 2.1 of
 * the License, or (at your option) any later version. See
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#ifndef __TILTING_LIDAR_SCANNER_HPP__
#define __TILTING_LIDAR_SCANNER_HPP__

#include "ros/ros.h"
#include <mutex>
#include <SerialStream.h>
#include <SerialStreamBuf.h>

namespace tiltingLIDARScanner {

/*!
 * A class that provides the main method for launching a ControlIt!
 * controller.
 */
class TiltingLIDARScanner
{
public:
    /*!
     * The default constructor.
     */
    TiltingLIDARScanner();

    /*!
     * The destructor.
     */
    virtual ~TiltingLIDARScanner();

    /*!
     * Initializes the tilting LIDAR scanner.
     *
     * \return Whether the initialization was successful.
     */
    bool init();

    /*!
     * Starts the tilting LIDAR scanner.
     *
     * \return Whether the execution was successful.
     */
    bool start();

    /*!
     * Stops the tilting LIDAR sensor. This should be called
     * before the program exist.
     */
    bool stop();

private:
    /*!
     * The callback function for the subscription to laser scan messages.
     * It stores the laser scan information in the laserScan member variable so that
     * it can be accessed in a real-time-safe manner.
     *
     * \param scan A message containing the laser scan data.
     */
    void laserScanCallback(const boost::shared_ptr<sensor_msgs::LaserScan const> & scan);

    /*!
     * The ROS node handle.
     */
    ros::NodeHandle nh;

    /*!
     * The point cloud slice publisher and point cloud publisher.
     */
    ros::Publisher slicePublisher, pcPublisher;

    /*!
     * The serial port on which the Arduino is connected.
     */
    std::string serialPortName;

    /*!
     * The name of the ROS topic on which the laser scan is being published.
     */
    std::string laserScanTopic;

    /*!
     * The serial port connection to the Arduino.
     */
    LibSerial::SerialStream serialPort;

    /*!
     * The ROS topic subscriber for laser scan data.
     */
    ros::Subscriber laserScanSubscriber;

    /*!
     * Stores the most recently received laser scan.
     */
    sensor_msgs::LaserScan laserScan;

    /*!
     * The command.
     */
    int cmd;

    /*!
     * Mutexes for protecting variables that are shared between the ROS topic
     * subscriber callback thread and the main thread.
     */
    std::mutex scanMutex, cmdMutex;
};

} // namespace tiltingLIDARScanner

#endif // __TILTING_LIDAR_SCANNER_HPP__
