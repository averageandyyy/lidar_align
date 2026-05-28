# lidar_align

## A ROS 2 Humble method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose source

**Note: accurate results require highly non-planar motions, which makes the technique poorly suited for calibrating sensors mounted to cars.**

The method makes use of the property that point clouds from lidars appear more "crisp" when the calibration is correct. It does this as follows:

1. A transformation between the lidar and pose sensor is set.
2. The poses are used in combination with the above transformation to fuse all lidar points into a single point cloud.
3. The sum of the distances between each point and its nearest neighbors is computed.
4. An optimizer searches for the transformation that minimizes this distance.

## ROS 2 Humble migration notes

This repository has been migrated from ROS 1 to **ROS 2 Humble**.

The main behavior changes are:

- `catkin` / `roscpp` / `rosbag` were replaced by `ament_cmake` / `rclcpp` / `rosbag2_cpp`.
- The launch file is now `launch/lidar_align.launch.py` and should be run with `ros2 launch`.
- `input_bag_path` now expects a **ROS 2 bag** (a bag directory containing `metadata.yaml`, or a storage file such as `.db3` / `.mcap` when supported by rosbag2).
- The printed TF helper command now uses the ROS 2 `tf2_ros` static transform publisher.

If you still have a legacy ROS 1 `.bag`, convert it to a ROS 2 bag first, or use an external rosbag2 plugin that can read ROS 1 v2 bags.

## Installation

Install ROS 2 Humble and the system dependencies required by this package.

```bash
sudo apt update
sudo apt install \
  ros-humble-ros-base \
  ros-humble-pcl-conversions \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  ros-humble-rosbag2-cpp \
  libpcl-dev \
  libnlopt-cxx-dev
```

Then build the package inside a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# copy or clone lidar_align here
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select lidar_align
source install/setup.bash
```

## Running

### With transforms in the ROS 2 bag

```bash
ros2 launch lidar_align lidar_align.launch.py \
  bag_file:=/path/to/rosbag2_directory
```

### With transforms in a Maplab CSV file

```bash
ros2 launch lidar_align lidar_align.launch.py \
  bag_file:=/path/to/rosbag2_directory \
  transforms_from_csv:=true \
  csv_file:=/path/to/poses.csv
```

By default, output files are written into `/tmp/lidar_align_results/`. You can override them:

```bash
ros2 launch lidar_align lidar_align.launch.py \
  bag_file:=/path/to/rosbag2_directory \
  output_pointcloud_path:=/tmp/aligned_points.ply \
  output_calibration_path:=/tmp/calibration.txt
```

## Input transformations

The final calibration quality is strongly correlated with the quality of the transformation source and the range of motion observed. To ensure an accurate calibration the dataset should encompass a large range of rotations and translations. Motion that is approximately planar (for example a car driving down a street) does not provide information about the system in the direction perpendicular to the plane, which can cause the optimizer to return incorrect estimates in that direction.

## Estimation procedure

For most systems the node can be run without tuning the parameters. By default, two optimizations are performed: a rough angle-only global optimization followed by a local 6-dof refinement.

The node loads all messages of type `sensor_msgs/msg/PointCloud2` from the given ROS 2 bag for use as lidar scans. The poses can either be given in the same bag file as `geometry_msgs/msg/TransformStamped` messages or in a separate CSV file that follows the format of [Maplab](https://github.com/ethz-asl/maplab).

## Visualization and results

The node outputs its current estimated transform while running. Use `output='screen'` in the launch file (already set in the provided launch file) to see the optimization progress.

When the optimization finishes, the transformation parameters are printed to the console. Example output:

```text
Active Transformation Vector (x,y,z,rx,ry,rz) from the Pose Sensor Frame to the Lidar Frame:
[-0.0608575, -0.0758112, 0.27089, 0.00371254, 0.00872398, 1.60227]

Active Transformation Matrix from the Pose Sensor Frame to the Lidar Frame:
-0.031495  -0.999473   0.007832  -0.060858
 0.999499  -0.031470   0.003300  -0.075811
-0.003052   0.007932   0.999964   0.270890
 0.000000   0.000000   0.000000   1.000000

Active Translation Vector (x,y,z) from the Pose Sensor Frame to the Lidar Frame:
[-0.0608575, -0.0758112, 0.27089]

Active Hamiltonian Quaternion (w,x,y,z) from the Pose Sensor Frame to the Lidar Frame:
[0.69588, 0.00166397, 0.00391012, 0.718145]

Time offset that must be added to lidar timestamps in seconds:
0.00594481

ROS 2 static TF publisher:
ros2 run tf2_ros static_transform_publisher --x -0.0608575 --y -0.0758112 --z 0.27089 --qx 0.00166397 --qy 0.00391012 --qz 0.718145 --qw 0.69588 --frame-id POSE_FRAME --child-frame-id LIDAR_FRAME
```

If the output paths are set, the calibration will also be saved to a text file and the aligned lidar points will be written to a PLY file.

## CSV format

| Column | Description |
|--:|:--|
| 1 | timestamp ns |
| 2 | vertex index (not used) |
| 3 | position x |
| 4 | position y |
| 5 | position z |
| 6 | orientation quaternion w |
| 7 | orientation quaternion x |
| 8 | orientation quaternion y |
| 9 | orientation quaternion z |

Note that Maplab has two CSV exporters. This file format matches the output of `exportPosesVelocitiesAndBiasesToCsv`, but differs from `exportVerticesAndTracksToCsv`.

## Parameters

### Scan parameters

| Parameter | Description | Default |
| -------------------- | :-----------: | :-------: |
| `min_point_distance` | Minimum range a point can be from the lidar and still be included in the optimization. | 0.0 |
| `max_point_distance` | Maximum range a point can be from the lidar and still be included in the optimization. | 100.0 |
| `keep_points_ratio` | Ratio of points to use in the optimization. Runtime increases drastically as this is increased. | 0.01 |
| `min_return_intensity` | The minimum return intensity a point requires to be considered valid. | -1.0 |
| `motion_compensation` | If the movement of the lidar during a scan should be compensated for. | true |
| `estimate_point_times` | Uses point angle together with `lidar_rpm` and `clockwise_lidar` to estimate the time a point was taken at. | false |
| `lidar_rpm` | Spin rate of the lidar in rpm, only used with `estimate_point_times`. | 600 |
| `clockwise_lidar` | True if the lidar spins clockwise, false for anti-clockwise. | false |

### I/O parameters

| Parameter | Description | Default |
| -------------------- | :-----------: | :-------: |
| `use_n_scans` | Optimization will only be run on the first `n` scans of the dataset. | 2147483647 |
| `input_bag_path` | Path of the ROS 2 bag containing `sensor_msgs/msg/PointCloud2` messages from the lidar. | N/A |
| `transforms_from_csv` | True to load poses from a CSV file, false to load them from the ROS 2 bag. | false |
| `input_csv_path` | Path of a CSV generated by Maplab, giving poses of the reference system. | N/A |
| `output_pointcloud_path` | If set, a fused point cloud will be saved to this path as a PLY when the calibration finishes. | launch default |
| `output_calibration_path` | If set, a text document giving the final transform will be saved to this path. | launch default |

### Aligner parameters

| Parameter | Description | Default |
| -------------------- | :-----------: | :-------: |
| `local` | If false, a global optimization is performed and the result is used instead of `inital_guess`. | false |
| `inital_guess` | Initial guess to the calibration `(x, y, z, rx, ry, rz, time_offset)`, only used in local mode. The historical parameter name is intentionally preserved for compatibility. | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` |
| `max_time_offset` | Maximum time offset between sensor clocks in seconds. | 0.1 |
| `angular_range` | Search range in radians around the `inital_guess` during the local optimization stage. | 0.5 |
| `translation_range` | Search range in meters around the `inital_guess` during the local optimization stage. | 1.0 |
| `max_evals` | Maximum number of function evaluations to run. | 200 |
| `xtol` | Tolerance of the final solution. | 0.0001 |
| `knn_batch_size` | Number of points to send to each thread when finding nearest points. | 1000 |
| `knn_k` | Number of neighbors to consider in the error function. | 1 |
| `global_knn_max_dist` | Error between points is limited to this value during global optimization. | 1.0 |
| `local_knn_max_dist` | Error between points is limited to this value during local optimization. | 0.1 |
| `time_cal` | True to perform time offset calibration. | true |

# Running with initial guess
```bash
ros2 launch lidar_align lidar_align.launch.py   bag_file:=/home/griffinlabs/workspaces/ros_ws/tf_and_right_cloud_2/   output_pointcloud_path:=/home/griffinlabs/workspaces/ros_ws/aligned.ply   output_calibration_path:=/home/griffinlabs/workspaces/ros_ws/calib.txt   local:=true   inital_guess:=[0.0,-0.14,1.9,-1.8391887,1.8392001,0.6332774,0.0]
# Initial Guess derived from V3C URDF

```
