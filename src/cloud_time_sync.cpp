#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class CloudTimeSyncNode : public rclcpp::Node
{
public:
    CloudTimeSyncNode()
        : Node("cloud_time_sync_node"),
          offset_inited_(false),
          offset_(0, 0)
    {
        this->declare_parameter<std::string>("input_topic", "/utlidar/cloud");
        this->declare_parameter<std::string>("output_topic", "/utlidar/cloud_sync");
        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);

        last_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, rclcpp::QoS(rclcpp::KeepLast(100)));
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS(),
            std::bind(&CloudTimeSyncNode::cloud_callback, this, std::placeholders::_1));
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        rclcpp::Time msg_stamp(msg->header.stamp, this->get_clock()->get_clock_type());
        rclcpp::Time now = this->get_clock()->now();
        rclcpp::Time out_stamp = now;
        if (msg_stamp.nanoseconds() > 0) {
            if (!offset_inited_) {
                offset_ = now - msg_stamp;
                offset_inited_ = true;
            }
            out_stamp = msg_stamp + offset_;
        }
        if (out_stamp.nanoseconds() <= last_stamp_.nanoseconds()) {
            out_stamp = rclcpp::Time(last_stamp_.nanoseconds() + 1, this->get_clock()->get_clock_type());
        }
        last_stamp_ = out_stamp;

        auto out = std::make_unique<sensor_msgs::msg::PointCloud2>(*msg);
        out->header.stamp = out_stamp;
        pub_->publish(std::move(out));
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    std::string input_topic_;
    std::string output_topic_;
    bool offset_inited_;
    rclcpp::Duration offset_;
    rclcpp::Time last_stamp_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudTimeSyncNode>());
    rclcpp::shutdown();
    return 0;
}
