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

#include "sensor_msgs/PointCloud2.h"
#include "tilting_lidar_scanner/PointCloudSliceMsg.h"
#include <tf/transform_broadcaster.h>

#include <Coordinate.hpp>
#include <PointCloud2MsgCreator.hpp>

#include <thread>
#include <mutex>

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
    bool addSlice(const tilting_lidar_scanner::PointCloudSliceMsg & slice);

private:

    /*!
     * This method is executed by a separate thread. It creates a
     * point cloud based on slice information.
     */
    void processSlices();

    /*!
     * Processes a slice by computing the coordinates of the points in the 3D point cloud.
     *
     * \param currSlice The slice to process.
     */
    void processSlice(const tilting_lidar_scanner::PointCloudSliceMsg & currSlice);

    /*!
     * Convert an angle from radians to degrees.
     */
    double toDeg(double rad);

    /*!
     * Creates a PointCloud2 message from the list of 3D points and publishes
     * it onto a ROS topic.
     */
    void createAndPublishPointCloud();

    /*!
     * Whether the child thread is done and should exit.
     */
    bool done;

    /*!
     * The ROS node handle.
     */
    ros::NodeHandle nh;

    /*!
     * The name of the point cloud's frame.
     */
    std::string frameID;

    /*!
     * The conversions from the world frame to the point cloud's frame.
     */
    double xOffset, yOffset, zOffset, roll, pitch, yaw;

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

    /*!
     * The thread that updates the inactive ControlModel.
     */
    std::thread thread;

    /*!
     * A buffer of incoming slice messages.
     */
    std::vector<tilting_lidar_scanner::PointCloudSliceMsg> sliceBuff;

    /*!
     * A list of 3D points that are part of a point cloud.
     */
    std::vector<Coordinate> pointBuff;

    /*!
     * The object that converts from a list of 3D points into a point cloud.
     */
    PointCloud2MsgCreator pc2MsgCreator;

    /*!
     * Publishes the frame transformation from the world frame to
     * the tilting LIDAR sensor's point cloud's frame.
     */
    tf::TransformBroadcaster tfBroadcaster;

    /*!
     * The point cloud 2 message to publish.
     */
    sensor_msgs::PointCloud2 pc2Msg;

};

} // namespace tiltingLIDARScanner

#endif // __POINT_CLOUD_ASSEMBLER_HPP__
