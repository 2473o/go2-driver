#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <mutex>

// OpenCV & GStreamer
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp> 

template<typename T>
/**
 * @brief Declare and get a parameter from the ROS2 node.
 * @param node Pointer to the ROS2 node.
 * @param name Name of the parameter.
 * @param variable Reference to the variable where the parameter value will be stored.
 * @param default_value Default value to use if the parameter is not set.
 */
void declare_and_get_parameter(rclcpp::Node* node, std::string name, T& variable, const T& default_value) {
    node->declare_parameter<T>(name, default_value);
    node->get_parameter(name, variable);
}

class HeadCameraNode : public rclcpp::Node
{
public:
  HeadCameraNode()
  : Node("head_camera_node")
  {
    this->declare_parameter<bool>("pub_camera_raw_enable", false);
    this->declare_parameter<bool>("pub_camera_compressed_enable", false);
    this->declare_parameter<std::string>("network_interface", "eth0");
    this->declare_parameter<std::string>("pub_camera_topic", "NoYamlRead/Go2Camera");
    this->declare_parameter<std::string>("gst_pipeline", "");

    this->get_parameter("pub_camera_raw_enable", pub_camera_raw_enable_);
    this->get_parameter("pub_camera_compressed_enable", pub_camera_compressed_enable_);

    std::string network_if;
    this->get_parameter("network_interface", network_if);

    std::string gst_pipeline;
    this->get_parameter("gst_pipeline", gst_pipeline);
    
    if (gst_pipeline.empty()) {
        RCLCPP_INFO(this->get_logger(), "gst_pipeline is empty");
    }

    std::string pub_camera_topic;
    this->get_parameter("pub_camera_topic", pub_camera_topic);

    if (pub_camera_compressed_enable_) {
      pub_camera_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        pub_camera_topic + "_Compressed", 10);
    }
    if (pub_camera_raw_enable_) {
      pub_camera_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
        pub_camera_topic + "_Raw", 10);
    }

    if (pub_camera_raw_enable_ || pub_camera_compressed_enable_) {
      cap_ = std::make_shared<cv::VideoCapture>(gst_pipeline, cv::CAP_GSTREAMER);
      if (!cap_->isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开视频流: %s", gst_pipeline.c_str());
        // Do not shutdown ROS immediately, let the node exist but report error.
        // If we want to exit, we should throw an exception instead of calling rclcpp::shutdown() in the constructor.
      } else {
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(30),
          std::bind(&HeadCameraNode::timer_callback, this)
        );
      }
    }

    RCLCPP_INFO(get_logger(), "head_camera_node started");
  }

private:
  
  std::mutex time_mutex_;
  rclcpp::Time last_global_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Time GetMonotonicNow()
  {
      std::lock_guard<std::mutex> lock(time_mutex_);
      rclcpp::Time now = this->get_clock()->now();
      
      if (now <= last_global_stamp_) {
          now = rclcpp::Time(last_global_stamp_.nanoseconds() + 1, RCL_ROS_TIME);
      }
      last_global_stamp_ = now;
      return now;
  }

  void timer_callback()
  {
    cv::Mat frame;
    if (cap_->read(frame)) {
      if (pub_camera_raw_enable_) {
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp = GetMonotonicNow();
        msg.header.frame_id = "camera";
        msg.height = frame.rows;
        msg.width  = frame.cols;
        msg.encoding     = "bgr8";
        msg.is_bigendian = false;
        msg.step         = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        msg.data.assign(frame.datastart, frame.dataend);
        pub_camera_raw_->publish(msg);
      }

      if (pub_camera_compressed_enable_) {
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(256, 256));
        std::vector<uchar> buf;
        cv::imencode(".jpg", resized, buf);
        sensor_msgs::msg::CompressedImage cim;
        cim.header.stamp    = GetMonotonicNow();
        cim.header.frame_id = "camera";
        cim.format = "jpeg";
        cim.data   = std::move(buf);
        pub_camera_compressed_->publish(cim);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "采集视频帧失败");
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_camera_raw_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_camera_compressed_;
  std::shared_ptr<cv::VideoCapture> cap_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool pub_camera_raw_enable_;
  bool pub_camera_compressed_enable_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HeadCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}