#ifndef POINTCLOUD_TO_LASERSCAN__CUPCL_IMPL_HPP_
#define POINTCLOUD_TO_LASERSCAN__CUPCL_IMPL_HPP_

#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "sensor_msgs/msg/point_cloud2.hpp"

#ifdef GO2_PERCEPTION_HAS_CUPCL
#include "cuda_runtime.h"
#include "lib/cudaFilter.h"
#endif

namespace pointcloud_to_laserscan
{

class CupclContext
{
public:
  CupclContext()
  {
#ifdef GO2_PERCEPTION_HAS_CUPCL
    int device_count = 0;
    const cudaError_t device_count_status = cudaGetDeviceCount(&device_count);
    if (device_count_status != cudaSuccess) {
      init_error_ = cudaGetErrorString(device_count_status);
      return;
    }
    if (device_count <= 0) {
      init_error_ = "cudaGetDeviceCount returned 0 device";
      return;
    }
    const cudaError_t stream_status = cudaStreamCreate(&stream_);
    if (stream_status != cudaSuccess) {
      init_error_ = cudaGetErrorString(stream_status);
      stream_ = nullptr;
      return;
    }
    filter_ = std::make_unique<cudaFilter>(stream_);
    ready_ = true;
#else
    init_error_ = "binary built without GO2_PERCEPTION_HAS_CUPCL";
#endif
  }

  ~CupclContext()
  {
#ifdef GO2_PERCEPTION_HAS_CUPCL
    if (device_input_ != nullptr) {
      cudaFree(device_input_);
    }
    if (device_output_ != nullptr) {
      cudaFree(device_output_);
    }
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
#endif
  }

  bool ready() const
  {
    return ready_;
  }

  const std::string & initError() const
  {
    return init_error_;
  }

  const std::string & lastError() const
  {
    return last_error_;
  }

  bool filter(
    const sensor_msgs::msg::PointCloud2 & cloud,
    double min_height, double max_height,
    double voxel_leaf_size,
    std::vector<float> & filtered_points)
  {
#ifdef GO2_PERCEPTION_HAS_CUPCL
    last_error_.clear();
    const PointFieldOffsets offsets = getPointFieldOffsets(cloud);
    if (!offsets.valid()) {
      last_error_ = "point cloud has no float32 x/y/z field";
      return false;
    }

    const std::size_t point_count = getPointCount(cloud);
    if (!ensureCapacity(point_count)) {
      return false;
    }
    if (!uploadCloud(cloud, offsets, point_count)) {
      return false;
    }

    unsigned int filtered_count = static_cast<unsigned int>(point_count);
    float * final_buffer = device_input_;
    if (passthrough_enabled_) {
      FilterParam_t passthrough_params{};
      passthrough_params.type = PASSTHROUGH;
      passthrough_params.dim = 2;
      passthrough_params.upFilterLimits = static_cast<float>(max_height);
      passthrough_params.downFilterLimits = static_cast<float>(min_height);
      passthrough_params.limitsNegative = false;
      if (filter_->set(passthrough_params) != 0) {
        passthrough_enabled_ = false;
      } else {
        filtered_count = 0;
        if (filter_->filter(device_output_, &filtered_count, device_input_, point_count) != 0) {
          last_error_ = "cudaFilter::filter(PASSTHROUGH) failed";
          return false;
        }
        final_buffer = device_output_;
      }
    }

    if (voxel_leaf_size > 0.0) {
      FilterParam_t voxel_params{};
      voxel_params.type = VOXELGRID;
      voxel_params.voxelX = static_cast<float>(voxel_leaf_size);
      voxel_params.voxelY = static_cast<float>(voxel_leaf_size);
      voxel_params.voxelZ = static_cast<float>(voxel_leaf_size);
      if (filter_->set(voxel_params) != 0) {
        last_error_ = "cudaFilter::set(VOXELGRID) failed";
        return false;
      }
      float * voxel_output = final_buffer == device_input_ ? device_output_ : device_input_;
      if (filter_->filter(voxel_output, &filtered_count, final_buffer, filtered_count) != 0) {
        last_error_ = "cudaFilter::filter(VOXELGRID) failed";
        return false;
      }
      final_buffer = voxel_output;
    }

    filtered_points.resize(static_cast<std::size_t>(filtered_count) * kPointStride);
    if (filtered_points.empty()) {
      return true;
    }

    if (cudaMemcpyAsync(
        filtered_points.data(),
        final_buffer,
        filtered_points.size() * sizeof(float),
        cudaMemcpyDeviceToHost,
        stream_) != cudaSuccess)
    {
      last_error_ = "cudaMemcpyAsync device->host failed";
      return false;
    }

    const cudaError_t sync_status = cudaStreamSynchronize(stream_);
    if (sync_status != cudaSuccess) {
      last_error_ = cudaGetErrorString(sync_status);
      return false;
    }
    return true;
#else
    (void)cloud;
    (void)min_height;
    (void)max_height;
    (void)voxel_leaf_size;
    filtered_points.clear();
    return false;
#endif
  }

private:
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

  static constexpr std::size_t kPointStride = 4;
  static constexpr std::size_t kPointStepBytes = sizeof(float) * kPointStride;

  static std::size_t getPointCount(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    return static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  }

  static PointFieldOffsets getPointFieldOffsets(const sensor_msgs::msg::PointCloud2 & cloud)
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

  static float readFloatField(const uint8_t * point_data, std::size_t offset)
  {
    float value = 0.0f;
    std::memcpy(&value, point_data + offset, sizeof(float));
    return value;
  }

  bool ensureCapacity(std::size_t point_count)
  {
#ifdef GO2_PERCEPTION_HAS_CUPCL
    if (point_count <= capacity_) {
      return true;
    }

    if (device_input_ != nullptr) {
      cudaFree(device_input_);
      device_input_ = nullptr;
    }
    if (device_output_ != nullptr) {
      cudaFree(device_output_);
      device_output_ = nullptr;
    }

    const std::size_t byte_size = point_count * kPointStepBytes;
    const cudaError_t input_alloc_status = cudaMallocManaged(
      &device_input_, byte_size, cudaMemAttachHost);
    if (input_alloc_status != cudaSuccess) {
      last_error_ = cudaGetErrorString(input_alloc_status);
      return false;
    }
    const cudaError_t input_attach_status = cudaStreamAttachMemAsync(stream_, device_input_);
    if (input_attach_status != cudaSuccess) {
      cudaFree(device_input_);
      device_input_ = nullptr;
      last_error_ = cudaGetErrorString(input_attach_status);
      return false;
    }

    const cudaError_t output_alloc_status = cudaMallocManaged(
      &device_output_, byte_size, cudaMemAttachHost);
    if (output_alloc_status != cudaSuccess) {
      cudaFree(device_input_);
      device_input_ = nullptr;
      last_error_ = cudaGetErrorString(output_alloc_status);
      return false;
    }
    const cudaError_t output_attach_status = cudaStreamAttachMemAsync(stream_, device_output_);
    if (output_attach_status != cudaSuccess) {
      cudaFree(device_input_);
      device_input_ = nullptr;
      cudaFree(device_output_);
      device_output_ = nullptr;
      last_error_ = cudaGetErrorString(output_attach_status);
      return false;
    }

    capacity_ = point_count;
#else
    (void)point_count;
#endif
    return true;
  }

  bool uploadCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const PointFieldOffsets & offsets,
    std::size_t point_count)
  {
#ifdef GO2_PERCEPTION_HAS_CUPCL
    if (!cloud.is_bigendian &&
      offsets.x == 0 &&
      offsets.y == sizeof(float) &&
      offsets.z == sizeof(float) * 2 &&
      cloud.point_step == kPointStepBytes)
    {
      if (cudaMemcpyAsync(
          device_input_,
          cloud.data.data(),
          point_count * kPointStepBytes,
          cudaMemcpyHostToDevice,
          stream_) != cudaSuccess)
      {
        last_error_ = "cudaMemcpyAsync host->device failed (direct path)";
        return false;
      }
      const cudaError_t sync_status = cudaStreamSynchronize(stream_);
      if (sync_status != cudaSuccess) {
        last_error_ = cudaGetErrorString(sync_status);
        return false;
      }
      return true;
    }

    host_points_.resize(point_count * kPointStride);
    const uint8_t * point_data = cloud.data.data();
    for (std::size_t index = 0; index < point_count; ++index) {
      const uint8_t * point_ptr = point_data + index * cloud.point_step;
      const std::size_t output_offset = index * kPointStride;
      host_points_[output_offset] = readFloatField(point_ptr, offsets.x);
      host_points_[output_offset + 1] = readFloatField(point_ptr, offsets.y);
      host_points_[output_offset + 2] = readFloatField(point_ptr, offsets.z);
      host_points_[output_offset + 3] = 0.0f;
    }

    if (cudaMemcpyAsync(
        device_input_,
        host_points_.data(),
        host_points_.size() * sizeof(float),
        cudaMemcpyHostToDevice,
        stream_) != cudaSuccess)
    {
      last_error_ = "cudaMemcpyAsync host->device failed (repack path)";
      return false;
    }

    const cudaError_t sync_status = cudaStreamSynchronize(stream_);
    if (sync_status != cudaSuccess) {
      last_error_ = cudaGetErrorString(sync_status);
      return false;
    }
    return true;
#else
    (void)cloud;
    (void)offsets;
    (void)point_count;
    return false;
#endif
  }

#ifdef GO2_PERCEPTION_HAS_CUPCL
  cudaStream_t stream_{nullptr};
  std::unique_ptr<cudaFilter> filter_;
  float * device_input_{nullptr};
  float * device_output_{nullptr};
  std::size_t capacity_{0};
  std::vector<float> host_points_;
#endif
  bool ready_{false};
  std::string init_error_;
  std::string last_error_;
  bool passthrough_enabled_{true};
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN__CUPCL_IMPL_HPP_
