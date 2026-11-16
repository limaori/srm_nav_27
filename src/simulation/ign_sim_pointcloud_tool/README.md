# isaac_sim_pointcloud_tool

This package converts pointcloud in Ignition Gazebo to velodyne format. This is a ROS package. It subscribes to the LiDAR rostopic which is published by Ignition Gazebo. And it republish a LiDAR rostopoic in Velodyne format.

Some SLAM algorithm needs pointcloud in Velodyne format so that it can extract corner points. But Isaac ROS only send pointcloud contains XYZ information. This package helps to convert pointcloud to velodyne format.

`scan_period` defaults to 0.0 for Ignition GPU lidar: the complete cloud is sampled
at its header timestamp, so every point has a zero `time` offset. A nonzero value
synthesizes column-dependent offsets and should only be used with input that
actually models a sequential scan. Consumers must accept an explicit all-zero
`time` field without inferring rotating-lidar timing from azimuth.

For 32 vertical samples spanning -7 to 52 degrees, use
`ang_res_y: 1.903225806451613` (59 / 31). Both endpoints are included.
