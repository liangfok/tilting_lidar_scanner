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

#ifndef __POINT_CLOUD_ASSEMBLER_HPP__
#define __POINT_CLOUD_ASSEMBLER_HPP__

#include "ros/ros.h"

// #include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
// #include "std_msgs/Int32.h"
#include "pcg_node/PointCloudSliceMsg.h"

#include <mutex>
// #include <SerialStream.h>
// #include <SerialStreamBuf.h>

namespace tiltingLIDARScanner {

/*!
 * Creates a point cloud from slice information.
 */
class PointCloudAssembler
{
public:
    /*!
     * The default constructor.
     */
    PointCloudAssembler();

    /*!
     * The destructor.
     */
    virtual ~PointCloudAssembler();

    /*!
     * Initializes the point cloud assembler.
     *
     * \return Whether the initialization was successful.
     */
    bool init();

    /*!
     * Starts the point cloud assembler.
     *
     * \return Whether the start was successful.
     */
    bool start();

    /*!
     * Stops the point cloud assembler. This should be called
     * before the program exits.
     */
    bool stop();

    /*!
     * Adds a new point cloud slice to the point cloud.
     */
    bool addSlice(const pcg_node::PointCloudSliceMsg & slice);

private:

    /*!
     * Does the work of creating a point cloud from the slice
     * information.
     */
    void obtainSlice();

    /*!
     * The ROS node handle.
     */
    ros::NodeHandle nh;

    /*!
     * Mutexes for protecting variables that are shared between
     * multiple threads.
     */
    std::mutex sliceMutex;

    /*!
     * The message for holding the point cloud.
     */
    sensor_msgs::PointCloud2 pointCloudMsg;

    /*!
     * The point cloud publisher.
     */
    ros::Publisher pcPublisher;
};

} // namespace tiltingLIDARScanner

#endif // __POINT_CLOUD_ASSEMBLER_HPP__
