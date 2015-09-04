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

#ifndef __POINT_CLOUD_2_MSG_CREATOR_HPP__
#define __POINT_CLOUD_2_MSG_CREATOR_HPP__

#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "Coordinate.hpp"

#include <vector>

namespace tiltingLIDARScanner {

/*!
 * Creates a point cloud from slice information.
 */
class PointCloud2MsgCreator
{
public:
    /*!
     * The default constructor.
     */
    PointCloud2MsgCreator();

    /*!
     * The destructor.
     */
    virtual ~PointCloud2MsgCreator();

    /*!
     * Creates a point cloud 2 message from a bunch of points.
     *
     * \param frameID The name of the point cloud's frame.
     * \param point A vector of points.
     * \param pc2Msg The PointCloud2 message to store the data in.
     * \return Whether the message was successfully created.
     */
    bool createPointCloud2Msg(
        std::string & frameID,
        std::vector<Coordinate> points,
        sensor_msgs::PointCloud2 & pc2Msg);
};

} // namespace tiltingLIDARScanner

#endif // __POINT_CLOUD_2_MSG_CREATOR_HPP__
