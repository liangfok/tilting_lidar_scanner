# Tilting LIDAR Scanner
Contains the ROS node that controls with the tilting LIDAR scanner and generates a point cloud.

# Usage Notes

To start the nodes:

    $ roslaunch pcg_node TiltingLIDARScanner.launch

To make the tilting LIDAR scanner perform a recalibration routine:

    $ rostopic pub --once /tilting_lidar_scanner/cmd std_msgs/Int32 '{data: 2}'

To start point cloud generation:

    $ rostopic pub --once /tilting_lidar_scanner/cmd std_msgs/Int32 '{data: 1}'

To stop point cloud generation:

    $ rostopic pub --once /tilting_lidar_scanner/cmd std_msgs/Int32 '{data: 0}'