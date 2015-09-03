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

#ifndef __MCU_HPP__
#define __MCU_HPP__

#include <SerialStream.h>
#include <SerialStreamBuf.h>

namespace tiltingLIDARScanner {

/*!
 * Provides a software abstraction for the micro-controller.
 * In this case, the micro-controller is connected to the PC
 * via a serial connection.
 */
class MCU
{
public:
    /*!
     * The default constructor.
     */
    MCU();

    /*!
     * The destructor.
     */
    virtual ~MCU();

    /*!
     * Initializes the point cloud assembler.
     *
     * \return Whether the initialization was successful.
     */
    bool init();

    /*!
     * Closes the connection with the MCU.
     */
    bool stop();

    /*!
     * Tell the tilting stand to recalibrate itself.
     */
    bool recalibrate();

    /*!
     * Tells the MCU to take a step.
     */
    bool step(double & angle);

private:

    /*!
     * Issues a recalibrate command to the Arduino microcontroller.
     *
     * \param command The command to send.
     */
    void sendMCUCmd(char command);

    /*
     * Reads the serial port to get messages from the microcontroller.
     *
     * \return Whether a message from the microcontroller was
     * received.
     */
    bool rcvMCUMsg();

    /*!
     * The serial port on which the Arduino is connected.
     */
    std::string serialPortName;

    /*!
     * The serial port connection to the Arduino.
     */
    LibSerial::SerialStream serialPort;

    /*!
     * The command to send to the micro-processor.
     */
    char mcuCmd[2];
};

} // namespace tiltingLIDARScanner

#endif // __MCU_HPP__
