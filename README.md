# go2_driver

Unitree Go2 Robot Driver for ROS2

## Overview

This package provides ROS2 drivers for the Unitree Go2 quadruped robot, including:
- IMU and leg sensor data publishing
- Head camera streaming
- Velocity command control
- PointCloud2 to LaserScan conversion, with CPU fallback and optional cuPCL GPU filtering

## Supported ROS2 Versions

- Foxy (Ubuntu 20.04)
- Humble (Ubuntu 22.04)
- Jazzy (Ubuntu 24.04)

## Dependencies

- `rclcpp` - ROS2 C++ client library
- `unitree_go` - Unitree Go message definitions
- `unitree_api` - Unitree API for robot control
- `sensor_msgs` - Sensor message types
- `geometry_msgs` - Geometry message types
- `std_msgs` - Standard message types
- `message_filters`, `tf2_ros`, `tf2_sensor_msgs` - Point cloud transform and filtering support
- `laser_geometry` - Laser scan / point cloud utilities
- CUDA runtime and NVIDIA cuPCL - optional GPU acceleration for point cloud filtering
- OpenCV4 - Image processing

## Build

Default driver build:

```bash
cd ~/ros2
colcon build --merge-install --packages-select go2_driver
```

Build point cloud processing targets with cuPCL:

```bash
cd ~/ros2/src/go2_driver
source ./cupcl5.x.sh

cd ~/ros2
colcon build --merge-install --packages-select go2_driver --cmake-args -DBUILD_POINTCLOUD=ON
```

`cupcl5.x.sh` exports `CUPCL_PATH` and adds the cuPCL module libraries to `LD_LIBRARY_PATH`.
For JetPack 5.x deployments, edit the script to use the Jetson cuPCL path:

```bash
export CUPCL_PATH=/home/unitree/ros2_ws/3rdparty/cuPCL
```

You can also override the cuPCL root directly at configure time:

```bash
colcon build --merge-install --packages-select go2_driver \
  --cmake-args -DBUILD_POINTCLOUD=ON -DCUPCL_ROOT=/path/to/cuPCL
```

## Nodes

### lowstate_driver

Publishes IMU and leg sensor data from the robot's LowState messages.
README.md
|-----------|------|---------|-------------|
| `imu_enable` | bool | `true` | Enable IMU publishing |
| `leg_sensor_enable` | bool | `false` | Enable leg sensor publishing |
| `imu_topic` | string | `/imu` | IMU topic name |
| `leg_sensor_topic` | string | `/leg_sensor` | Leg sensor topic name |
| `use_sim_time` | bool | `false` | Use simulation time |

**Published Topics:**

- `/imu` (sensor_msgs/Imu) - IMU data
- `/leg_sensor` (go2_driver/LegSensor) - Leg sensor data

### head_camera

Streams head camera video using GStreamer and publishes as ROS2 image topics.

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pub_camera_raw_enable` | bool | `false` | Enable raw image publishing |
| `pub_camera_compressed_enable` | bool | `false` | Enable compressed image publishing |
| `pub_camera_topic` | string | `/head_camera/image_raw` | Image topic base name |
| `network_interface` | string | `eth0` | Network interface for UDP stream |
| `gst_pipeline` | string | - | GStreamer pipeline string |

**Published Topics:**

- `<topic>_Raw` (sensor_msgs/Image) - Raw image
- `<topic>_Compressed` (sensor_msgs/CompressedImage) - Compressed image

### sport_driver

Converts geometry_msgs/Twist commands to Unitree Sport API requests for robot velocity control.

**Subscribed Topics:**

- `/cmd_vel` (geometry_msgs/Twist) - Velocity command
- `/sportmodestate` (unitree_go/SportModeState) - Robot sport mode state

**Published Topics:**

- `/api/sport/request` (unitree_api/Request) - Sport API request

### pointcloud_to_laserscan_node

Converts a PointCloud2 stream to LaserScan. The node can use cuPCL for GPU PassThrough/VoxelGrid filtering when `BUILD_POINTCLOUD=ON`, `CUPCL_PATH` or `CUPCL_ROOT` points to cuPCL, and `use_cupcl` is enabled. If cuPCL is unavailable at runtime, it falls back to the CPU path.

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `target_frame` | string | `""` | Target TF frame for cloud projection |
| `transform_tolerance` | double | `0.01` | TF lookup timeout in seconds |
| `min_height` | double | system min | Minimum point height |
| `max_height` | double | system max | Maximum point height |
| `angle_min` | double | `-pi` | Minimum scan angle |
| `angle_max` | double | `pi` | Maximum scan angle |
| `angle_increment` | double | `pi / 180` | Scan angular resolution |
| `range_min` | double | `0.0` | Minimum valid range |
| `range_max` | double | system max | Maximum valid range |
| `use_inf` | bool | `true` | Use infinity for empty scan bins |
| `queue_size` | int | `1` | Point cloud queue depth; keep low for real-time use |
| `qos_reliable` | bool | `false` | Use reliable QoS instead of best-effort sensor QoS |
| `max_cloud_age` | double | `0.3` | Drop stale cloud frames older than this many seconds; `0` disables dropping |
| `max_points_per_scan` | int | `80000` | Maximum points projected per scan; `0` disables subsampling |
| `use_cupcl` | bool | build-dependent | Enable cuPCL GPU filtering |
| `voxel_leaf_size` | double | `0.02` | cuPCL VoxelGrid leaf size; `0` disables voxel filtering |
| `cupcl_retry_without_voxel` | bool | `true` | Retry GPU path without VoxelGrid if VoxelGrid fails |

**Subscribed Topics:**

- `/rslidar_points` (sensor_msgs/PointCloud2) - Input point cloud, remapped from `cloud_in`

**Published Topics:**

- `/scan` (sensor_msgs/LaserScan) - Output scan

## Launch Files

### driver.launch.py

Main launch file that starts all driver nodes:

```bash
ros2 launch go2_driver driver.launch.py
```

**Launch Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `sim` | `false` | Use simulation time |
| `odom` | `false` | Start odom-to-path publishing |

**Includes:**

- lowstate_driver node
- sport_driver node
- multi_static_tf.py node
- odom_to_path.py node when `odom:=true`

### pointcloud.launch.py

Launches PointCloud2 to LaserScan conversion:

```bash
ros2 launch go2_driver pointcloud.launch.py sim:=true use_cupcl:=true
```

**Launch Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `sim` | `false` | Use simulation time |
| `use_cupcl` | `true` | Enable cuPCL if the binary was built with cuPCL support |
| `queue_size` | `1` | PointCloud2 queue depth |
| `qos_reliable` | `false` | Use reliable QoS instead of best-effort |
| `max_cloud_age` | `0.3` | Drop old point clouds to avoid accumulating latency |
| `max_points_per_scan` | `80000` | Limit projected points per scan for large clouds |

For very large point clouds, lower the point cap and stale-frame threshold:

```bash
ros2 launch go2_driver pointcloud.launch.py \
  sim:=true use_cupcl:=true max_points_per_scan:=40000 max_cloud_age:=0.2
```

## Message Types

### LegSensor (go2_driver/LegSensor)

```
uint64 timestamp_ns     # Timestamp in nanoseconds
float32[12] q           # Joint positions
float32[12] dq          # Joint velocities
float32[12] tau         # Estimated joint torques
int16[4] foot_force     # Foot force sensors
float32[4] imu_quaternion   # IMU quaternion [w,x,y,z]
float32[3] imu_gyroscope   # IMU gyroscope data
float32[3] imu_accelerometer # IMU accelerometer data
```

## Configuration Example

```yaml
# config.yaml
/**:
  ros__parameters:
    use_sim_time: false

/lowstate_driver:
  ros__parameters:
    imu_enable: true
    leg_sensor_enable: true
    imu_topic: /imu
    leg_sensor_topic: /leg_sensor

/head_camera:
  ros__parameters:
    pub_camera_raw_enable: true
    pub_camera_compressed_enable: true
    pub_camera_topic: /head_camera/image_raw
    network_interface: eth0
    gst_pipeline: >
      udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 !
      application/x-rtp,media=video,encoding-name=H264 !
      rtph264depay ! h264parse ! nvv4l2decoder enable-max-performance=1 !
      nvvidconv output-buffers=1 ! video/x-raw,format=BGRx,width=1280,height=720 !
      videoconvert ! video/x-raw,format=BGR ! appsink drop=1 sync=false
```

## Troubleshooting

### Camera stream not opening

Check network interface and firewall settings:
```bash
# Verify network interface
ip addr show eth0

# Check if UDP port 1720 is open
sudo iptables -L -n | grep 1720
```

### IMU data not publishing

Verify LowState topic is being published by the robot:
```bash
ros2 topic hz /lowstate
```

### cuPCL not enabled

Build with point cloud targets enabled and source the cuPCL environment before building:
```bash
source ~/ros2/src/go2_driver/cupcl5.x.sh
cd ~/ros2
colcon build --merge-install --packages-select go2_driver --cmake-args -DBUILD_POINTCLOUD=ON
```

At launch, the node logs whether cuPCL is active:

```text
cuPCL 已启用，点云高度/体素过滤将使用 GPU
```

If you see `no CUDA-capable device is detected`, either run on a machine with an NVIDIA GPU/CUDA driver or disable GPU filtering:

```bash
ros2 launch go2_driver pointcloud.launch.py use_cupcl:=false
```

You can confirm the process is using the GPU with:

```bash
nvidia-smi
```

## License

BSD-3-Clause
