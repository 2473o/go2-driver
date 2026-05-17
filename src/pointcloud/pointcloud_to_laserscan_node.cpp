/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#ifdef _MSC_VER
#define _USE_MATH_DEFINES
#endif

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"
#include "pointcloud_to_laserscan/cupcl_impl.hpp"

#include <cmath>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if __has_include("tf2_sensor_msgs/tf2_sensor_msgs.h")
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#elif __has_include("tf2_sensor_msgs/tf2_sensor_msgs.hpp")
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#else
#endif

#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace pointcloud_to_laserscan
{
namespace
{

constexpr std::size_t kCupclPointStride = 4;

struct PointFieldOffsets
{
  std::size_t x{std::numeric_limits<std::size_t>::max()};
  std::size_t y{std::numeric_limits<std::size_t>::max()};
  std::size_t z{std::numeric_limits<std::size_t>::max()};

  bool valid() const
  {
    return x != std::numeric_limits<std::size_t>::max() &&
      y != std::numeric_limits<std::size_t>::max() &&
      z != std::numeric_limits<std::size_t>::max();
  }
};

struct ScanProjectionParams
{
  float min_height;
  float max_height;
  float range_min_sq;
  float range_max_sq;
  float angle_min;
  float angle_max;
  float inv_angle_increment;
};

std::size_t getPointCount(const sensor_msgs::msg::PointCloud2 & cloud)
{
  return static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
}

PointFieldOffsets getPointFieldOffsets(const sensor_msgs::msg::PointCloud2 & cloud)
{
  PointFieldOffsets offsets;
  for (const auto & field : cloud.fields) {
    if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
      continue;
    }
    if (field.name == "x") {
      offsets.x = field.offset;
    } else if (field.name == "y") {
      offsets.y = field.offset;
    } else if (field.name == "z") {
      offsets.z = field.offset;
    }
  }
  return offsets;
}

float readFloatField(const uint8_t * point_data, std::size_t offset)
{
  float value = 0.0f;
  std::memcpy(&value, point_data + offset, sizeof(float));
  return value;
}

void updateLaserScanFromPoint(
  float x, float y, float z,
  sensor_msgs::msg::LaserScan & scan_msg,
  const ScanProjectionParams & params)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return;
  }

  if (z < params.min_height || z > params.max_height) {
    return;
  }

  const float range_sq = x * x + y * y;
  if (range_sq < params.range_min_sq || range_sq > params.range_max_sq) {
    return;
  }

  const float angle = std::atan2(y, x);
  if (angle < params.angle_min || angle > params.angle_max) {
    return;
  }

  const std::size_t index = static_cast<std::size_t>(
    (angle - params.angle_min) * params.inv_angle_increment);
  if (index >= scan_msg.ranges.size()) {
    return;
  }

  const float current_range = scan_msg.ranges[index];
  if (range_sq < current_range * current_range) {
    scan_msg.ranges[index] = std::sqrt(range_sq);
  }
}

void accumulateLaserScanFromCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const PointFieldOffsets & offsets,
  sensor_msgs::msg::LaserScan & scan_msg,
  const ScanProjectionParams & params,
  std::size_t point_sample_step)
{
  const std::size_t point_count = getPointCount(cloud);
  const uint8_t * point_data = cloud.data.data();
  point_sample_step = std::max<std::size_t>(point_sample_step, 1);

  for (std::size_t index = 0; index < point_count; index += point_sample_step) {
    const uint8_t * point_ptr = point_data + index * cloud.point_step;
    updateLaserScanFromPoint(
      readFloatField(point_ptr, offsets.x),
      readFloatField(point_ptr, offsets.y),
      readFloatField(point_ptr, offsets.z),
      scan_msg, params);
  }
}

void accumulateLaserScanFromFloat4(
  const std::vector<float> & points,
  sensor_msgs::msg::LaserScan & scan_msg,
  const ScanProjectionParams & params,
  std::size_t point_sample_step)
{
  point_sample_step = std::max<std::size_t>(point_sample_step, 1);
  const std::size_t data_step = kCupclPointStride * point_sample_step;
  for (std::size_t index = 0; index + 3 < points.size(); index += data_step) {
    updateLaserScanFromPoint(
      points[index],
      points[index + 1],
      points[index + 2],
      scan_msg,
      params);
  }
}

}  // namespace

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  target_frame_ = this->declare_parameter("target_frame", "");
  tolerance_ = this->declare_parameter("transform_tolerance", 0.01);
  // TODO(hidmic): adjust default input queue size based on actual concurrency levels
  // achievable by the associated executor
  // input_queue_size_ = this->declare_parameter(
  //   "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
  input_queue_size_ = std::max<int>(
    1, static_cast<int>(this->declare_parameter<int>("queue_size", 1)));
  min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
  max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 10.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);
  qos_reliable_ = this->declare_parameter("qos_reliable", false);
  max_cloud_age_ = this->declare_parameter("max_cloud_age", 0.3);
  max_points_per_scan_ = this->declare_parameter<int>("max_points_per_scan", 80000);
#ifdef GO2_PERCEPTION_HAS_CUPCL
  use_cupcl_ = this->declare_parameter("use_cupcl", true);
#else
  use_cupcl_ = this->declare_parameter("use_cupcl", false);
#endif
  voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", 0.02);
  cupcl_retry_without_voxel_ = this->declare_parameter("cupcl_retry_without_voxel", true);
  cupcl_voxel_disabled_ = false;

  if (use_cupcl_) {
    cupcl_context_ = std::make_unique<CupclContext>();
    if (!cupcl_context_->ready()) {
      const std::string init_error = cupcl_context_->initError();
      cupcl_context_.reset();
      use_cupcl_ = false;
      RCLCPP_WARN(
        this->get_logger(),
        "cuPCL unavailable: %s, falling back to CPU pointcloud projection",
        init_error.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "cuPCL enabled, using GPU for pointcloud height/voxel filtering");
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "cuPCL disabled by parameter, using CPU pointcloud projection");
  }

  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", getSensorQos());

  using std::placeholders::_1;
  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);
    message_filter_ = std::make_unique<MessageFilter>(
      sub_, *tf2_, target_frame_, input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());
    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  } else {  // otherwise setup direct subscription
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));
  }

  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);
  if (subscription_listener_thread_.joinable()) {
    subscription_listener_thread_.join();
  }
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();
  rclcpp::QoS qos = getSensorQos();
  sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());

  const std::chrono::milliseconds timeout(100);
  while (rclcpp::ok(context) && alive_.load()) {
    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }
  sub_.unsubscribe();
}

rclcpp::QoS PointCloudToLaserScanNode::getSensorQos() const
{
  rclcpp::QoS qos(rclcpp::KeepLast(static_cast<std::size_t>(input_queue_size_)));
  if (qos_reliable_) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  qos.durability_volatile();
  return qos;
}

std::size_t PointCloudToLaserScanNode::getPointSampleStep(std::size_t point_count) const
{
  if (max_points_per_scan_ <= 0 ||
    point_count <= static_cast<std::size_t>(max_points_per_scan_))
  {
    return 1;
  }
  return (point_count + static_cast<std::size_t>(max_points_per_scan_) - 1) /
    static_cast<std::size_t>(max_points_per_scan_);
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  if (max_cloud_age_ > 0.0 &&
    (cloud_msg->header.stamp.sec != 0 || cloud_msg->header.stamp.nanosec != 0))
  {
    const double age = (this->now() - rclcpp::Time(cloud_msg->header.stamp)).seconds();
    if (age > max_cloud_age_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Pointcloud delayed by %.3fs, dropping old frame to prevent LaserScan latency accumulation",
        age);
      return;
    }
  }

  // build laserscan output
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;

  // // Override timestamp with current time
  // scan_msg->header.stamp = now();

  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  if (angle_increment_ <= 0.0 || angle_max_ <= angle_min_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "LaserScan angle parameters invalid, cannot convert pointcloud");
    return;
  }

  // determine amount of rays to create
  const std::size_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
  }

  // Transform cloud if necessary
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Transform failure: " << ex.what());
      return;
    }
  }

  const ScanProjectionParams projection_params{
    static_cast<float>(min_height_),
    static_cast<float>(max_height_),
    static_cast<float>(range_min_ * range_min_),
    static_cast<float>(range_max_ * range_max_),
    static_cast<float>(angle_min_),
    static_cast<float>(angle_max_),
    1.0f / static_cast<float>(angle_increment_)};

  if (use_cupcl_ && cupcl_context_ != nullptr) {
    std::lock_guard<std::mutex> cupcl_lock(cupcl_mutex_);
    cupcl_filtered_points_.clear();
    const double active_voxel_leaf_size = cupcl_voxel_disabled_ ? 0.0 : voxel_leaf_size_;
    if (cupcl_context_->filter(
        *cloud_msg,
        min_height_,
        max_height_,
        active_voxel_leaf_size,
        cupcl_filtered_points_))
    {
      accumulateLaserScanFromFloat4(
        cupcl_filtered_points_,
        *scan_msg,
        projection_params,
        getPointSampleStep(cupcl_filtered_points_.size() / kCupclPointStride));
      pub_->publish(std::move(scan_msg));
      return;
    }

    if (!cupcl_voxel_disabled_ && cupcl_retry_without_voxel_ && voxel_leaf_size_ > 0.0) {
      const std::string first_error = cupcl_context_->lastError();
      if (cupcl_context_->filter(
          *cloud_msg,
          min_height_,
          max_height_,
          0.0,
          cupcl_filtered_points_))
      {
        cupcl_voxel_disabled_ = true;
        RCLCPP_WARN(
          this->get_logger(),
          "cuPCL VoxelGrid failed(%s), auto-switched to GPU PassThrough mode",
          first_error.c_str());
        accumulateLaserScanFromFloat4(
          cupcl_filtered_points_,
          *scan_msg,
          projection_params,
          getPointSampleStep(cupcl_filtered_points_.size() / kCupclPointStride));
        pub_->publish(std::move(scan_msg));
        return;
      }
    }

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "cuPCL filter failed: %s, current frame falling back to CPU path",
      cupcl_context_->lastError().c_str());
  }

  const PointFieldOffsets offsets = getPointFieldOffsets(*cloud_msg);
  if (!offsets.valid()) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Pointcloud missing float32 x/y/z fields, cannot convert to LaserScan");
    return;
  }

  accumulateLaserScanFromCloud(
    *cloud_msg,
    offsets,
    *scan_msg,
    projection_params,
    getPointSampleStep(getPointCount(*cloud_msg)));

  pub_->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)
