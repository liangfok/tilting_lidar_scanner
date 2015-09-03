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

# Usage

To start the nodes:

    $ roslaunch pcg_node TiltingLIDARScanner.launch

To make the tilting LIDAR scanner perform a recalibration routine:

    $ rostopic pub --once /tilting_lidar_scanner/cmd std_msgs/Int32 '{data: 2}'

To start point cloud generation:

    $ rostopic pub --once /tilting_lidar_scanner/cmd std_msgs/Int32 '{data: 1}'

To stop point cloud generation:

    $ rostopic pub --once /tilting_lidar_scanner/cmd std_msgs/Int32 '{data: 0}'