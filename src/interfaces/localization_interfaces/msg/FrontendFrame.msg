# 一次已提交观测；点云按该时刻去畸变到 odometry.child_frame_id 坐标系。
uint64 instance_id
uint64 epoch
uint64 sequence
builtin_interfaces/Time begin
nav_msgs/Odometry odometry
sensor_msgs/PointCloud2 cloud
uint32 matches
bool lidar_updated
