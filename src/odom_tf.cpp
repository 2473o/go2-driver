#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::placeholders;

class OdomTfPublisherNode : public rclcpp::Node
{
public:
    OdomTfPublisherNode() : Node("odom_tf_publisher"), body_height_(0.30)
    {
        RCLCPP_INFO(this->get_logger(), "里程计&TF发布节点启动");

        // ========== 1. 声明并读取参数 ==========
        this->declare_parameter("publish_odom", true); // 声明里程计发布开关，默认true

        // 读取参数值
        if (!this->get_parameter("publish_odom", publish_odom_))
        {
            RCLCPP_WARN(this->get_logger(), "未读取到 publish_odom 参数，使用默认值: true");
            publish_odom_ = true;
        }

        this->declare_parameter("publish_tf", true);
        if (!this->get_parameter("publish_tf", publish_tf_))
        {
            RCLCPP_WARN(this->get_logger(), "未读取到 publish_tf 参数，使用默认值: true");
            publish_tf_ = true;
        }

        // 打印参数配置
        RCLCPP_INFO(this->get_logger(), "publish_odom=%s, publish_tf=%s",
                    publish_odom_ ? "true" : "false",
                    publish_tf_ ? "true" : "false");


        tf_bro_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sport_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", 10, std::bind(&OdomTfPublisherNode::state_cb, this, _1));


        robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/utlidar/robot_pose", 10, std::bind(&OdomTfPublisherNode::pose_callback, this, _1));

        if (publish_odom_)
        {
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        }
    }

private:
    // 核心成员变量
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bro_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_sub_;
    double body_height_; // 机身高度（暂未在pose回调中使用，保留原逻辑）

    // 参数变量：控制里程计的发布开关
    bool publish_odom_;
    bool publish_tf_;

    // 运动状态回调：更新机身高度
    void state_cb(const unitree_go::msg::SportModeState::SharedPtr state_msg)
    {
        // 原逻辑：修正机身高度
        body_height_ = state_msg->body_height + 0.057 - 0.046825;
    }

    // 雷达位姿回调：转换为base_link并根据参数发布TF/里程计
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 使用消息的时间戳，而不是当前时间，以避免TF_OLD_DATA错误并保持与数据源的一致性
        rclcpp::Time timestamp = msg->header.stamp;

        // 构造TF变换（odom → base_link）
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = timestamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        // 位姿修正：雷达→base_link
        transform.transform.translation.x = msg->pose.position.x - 0.28945;
        transform.transform.translation.y = msg->pose.position.y;
        transform.transform.translation.z = msg->pose.position.z + 0.046825;
        transform.transform.rotation = msg->pose.orientation;

        // ========== 根据参数决定是否发布TF ==========
        if (publish_tf_)
        {
            tf_bro_->sendTransform(transform);
        }

        // ========== 根据参数决定是否发布里程计 ==========
        if (publish_odom_)
        {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = timestamp;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = transform.transform.translation.x;
            odom.pose.pose.position.y = msg->pose.position.y;
            odom.pose.pose.position.z = transform.transform.translation.z;
            odom.pose.pose.orientation.x = msg->pose.orientation.x;
            odom.pose.pose.orientation.y = msg->pose.orientation.y;
            odom.pose.pose.orientation.z = msg->pose.orientation.z;
            odom.pose.pose.orientation.w = msg->pose.orientation.w;
            odom_pub_->publish(odom);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTfPublisherNode>());
    rclcpp::shutdown();
    return 0;
}