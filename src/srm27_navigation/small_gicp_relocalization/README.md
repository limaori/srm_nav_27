# small_gicp_relocalization

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/actions/workflows/build_and_test.yml/badge.svg?branch=main)](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization/actions/workflows/build_and_test.yml)

A simple example: Implementing point cloud alignment and localization using [small_gicp](https://github.com/koide3/small_gicp.git)

Given a registered pointcloud (based on the odom frame) and prior pointcloud (mapped using [pointlio](https://github.com/SMBU-PolarBear-Robotics-Team/Point-LIO) or similar tools), the node will calculate the transformation between the two point clouds and publish the correction from the `map` frame to the `odom` frame.

## Dependencies

- ROS2 Humble
- small_gicp
- pcl
- OpenMP

## Build

```zsh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization.git

cd ..
```

1. Install dependencies

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. Build

    ```zsh
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
    ```

## Usage

1. Set prior pointcloud file in [launch file](launch/small_gicp_relocalization_launch.py)

2. Adjust the transformation between `base_frame` and `lidar_frame`

    The `global_pcd_map` output by algorithms such as `pointlio` and `fastlio` is strictly based on the `lidar_odom` frame. However, the initial position of the robot is typically defined by the `base_link` frame within the `odom` coordinate system. To address this discrepancy, the code listens for the coordinate transformation from `base_frame`(velocity_reference_frame) to `lidar_frame`, allowing the `global_pcd_map` to be converted into the `odom` coordinate system.

    If not set, empty transformation will be used.

3. Run

    ```zsh
    ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py
    ```

## Registration and simulation

The input cloud must be in `odom_frame`. Each registration uses the latest scan,
downsampled by `registered_leaf_size`, at `registration_interval` seconds of ROS
time (default 0.5). Old scans are not accumulated while registration is busy.

Planar corrections use `atan2(R(1, 0), R(0, 0))` for yaw. Taking the third component
of Eigen's XYZ Euler decomposition can turn a small negative roll into a yaw near
180 degrees when roll and pitch are discarded.

An update must converge and pass the following checks after planar projection:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `max_translation_step` | 2.0 m | Maximum change from the previous accepted correction |
| `max_rotation_step` | 0.5 rad | Maximum rotation change from the previous accepted correction |
| `min_inlier_ratio` | 0.3 | Minimum fraction of source points within the correspondence distance |
| `max_registration_rmse` | 0.3 m | Maximum nearest-neighbor RMSE for those inliers |

The correction is not permanently bounded around `init_pose`: `map -> odom` must
be able to follow accumulated odometry drift. A rough starting pose is still
required because GICP performs local registration. RViz `2D Pose Estimate` resets
that pose using `robot_base_frame`; simulation uses `base_footprint`.

For the Ignition GPU lidar, set the converter's `scan_period` to 0.0. All points
are measured at the message timestamp. Point-LIO must honor the explicit `time`
field even when every offset is zero. Synthetic rotating-lidar offsets cause
false motion compensation during translation and turns. Use a PCD and occupancy
map from the same world and mapping origin; a previously distorted PCD may need
to be rebuilt after correcting the sensor timing.
