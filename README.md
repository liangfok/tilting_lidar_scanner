# Tilting LIDAR Scanner
Contains the ROS node that controls with the tilting LIDAR scanner and generates a point cloud.

# Workspace Configuration and Installation

To create a new ROS workspace and add this package to it:

    $ source /opt/ros/indigo/setup.bash
    $ mkdir -p ~/[workspace name]/src
    $ cd ~/[workspace name]/src
    $ catkin_init_workspace
    $ cd ..
    $ catkin_make
    $ cd ~/[workspace name]/src/
    $ git clone git@github.com:liangfok/pcg_node.git

Edit your ~/.bashrc and add the following line to the bottom of it:

    source $HOME/[workspace name]/devel/setup.bash

Before compiling, ensure the following dependencies are installed:

    $ sudo apt-get install libserial-dev

To compile:

    $ roscd
    $ cd ..
    $ catkin_make

# Usage

## Connect the Tilting Stand and LIDAR Sensor ##

First attach the Arduino's USB cable to your PC. Then attach the Hokuyo LIDAR sensor's USB cable to your PC. Normally, the Arduino's connection will be /dev/ttyACM0 and the Hokuyo LIDAR sensor's connection will be /dev/ttyACM1. You can verify this by looking at the kernel messages by executing "dmsg".

## Set Parameters ##

There are several parameters you can set like the serial ports used by the tilting stand and LIDAR sensor, the name of the point cloud's frame, and the conversion from the world frame to the point cloud's frame. These parameters are stored on the ROS parameter server and are loaded by the following launch file:


    ~/[workspace name]/src/pcg_node/launch/TiltingLIDARScanner.launch

Edit this launch file to have the desired parameter values.

## Start the Software ##

To start the tilting LIDAR scanner software:

    $ roslaunch pcg_node TiltingLIDARScanner.launch

## Using the Software ##

To recalibrate the tilting LIDAR scanner:

    $ rostopic pub --once /tilting_lidar_scanner/cmd std_msgs/Int32 '{data: 2}'

To start point cloud generation:

    $ rostopic pub --once /tilting_lidar_scanner/cmd std_msgs/Int32 '{data: 1}'

To stop point cloud generation:

    $ rostopic pub --once /tilting_lidar_scanner/cmd std_msgs/Int32 '{data: 0}'