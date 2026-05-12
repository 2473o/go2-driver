#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

// For real robot
#include "unitree_api/msg/request.hpp"
#include "go2_driver/sport_client.hpp"
#include "go2_driver/utilities.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

using namespace std::chrono_literals;

class SportClientCmdVel : public rclcpp::Node
{
public:
    SportClientCmdVel() : Node("sport_client_driver")
    {

        // 启用里程计默认false
        declare_and_get_parameter(this, "odom", odom_enabled_, false);

        // 初始化运动客户端
        sport_client_ = std::make_shared<SportClient>();

        // 多线程处理
        cmd_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions cmd_sub_opts;
        cmd_sub_opts.callback_group = cmd_cb_group_;

        // 如果是sim环境下，不会生效
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(10),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                handleVelocity(msg);
            },
            cmd_sub_opts);

        // 运动指令发布，sim环境下不会生效
        sport_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", rclcpp::QoS(10));

        if (odom_enabled_)
        {
            odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("robot_odom", rclcpp::QoS(10));

            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            tf_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            rclcpp::SubscriptionOptions tf_sub_opts;

            tf_sub_opts.callback_group = tf_cb_group_;

            tf_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
                "/sportmodestate", rclcpp::QoS(10).best_effort(),
                std::bind(&SportClientCmdVel::state_cb, this, std::placeholders::_1), tf_sub_opts);
        }
    }

private:
    // 处理速度命令
    void handleVelocity(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 直接转发速度命令
        unitree_api::msg::Request req;
        sport_client_->Move(req,
                            msg->linear.x,
                            msg->linear.y,
                            msg->angular.z);
        sport_pub_->publish(req);
    }

    // 处理机器人状态信息，更新高度并发布里程计
    void state_cb(const unitree_go::msg::SportModeState::SharedPtr state_msg)
    {
        double pos_x = state_msg->position[0];
        double pos_y = state_msg->position[1];

        const auto &quat = state_msg->imu_state.quaternion;
        tf2::Quaternion q(quat[1], quat[2], quat[3], quat[0]);

        auto stamp = this->get_clock()->now();

        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = stamp;
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_link";

        odom_msg->pose.pose.position.x = pos_x;
        odom_msg->pose.pose.position.y = pos_y;
        odom_msg->pose.pose.orientation.x = q.x();
        odom_msg->pose.pose.orientation.y = q.y();
        odom_msg->pose.pose.orientation.z = q.z();
        odom_msg->pose.pose.orientation.w = q.w();

        odom_msg->twist.twist.linear.x = state_msg->velocity[0];
        odom_msg->twist.twist.linear.y = state_msg->velocity[1];
        odom_msg->twist.twist.angular.z = state_msg->yaw_speed;

        std::fill(odom_msg->pose.covariance.begin(), odom_msg->pose.covariance.end(), 0.0);
        odom_msg->pose.covariance[0] = 0.01;
        odom_msg->pose.covariance[7] = 0.01;
        odom_msg->pose.covariance[14] = 0.01;
        odom_msg->pose.covariance[21] = 0.01;
        odom_msg->pose.covariance[28] = 0.01;
        odom_msg->pose.covariance[35] = 0.01;
        std::fill(odom_msg->twist.covariance.begin(), odom_msg->twist.covariance.end(), 0.0);
        odom_msg->twist.covariance[0] = 0.01;
        odom_msg->twist.covariance[7] = 0.01;
        odom_msg->twist.covariance[14] = 0.01;
        odom_msg->twist.covariance[21] = 0.01;
        odom_msg->twist.covariance[28] = 0.01;
        odom_msg->twist.covariance[35] = 0.01;

        odom_pub_->publish(std::move(odom_msg));

        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = stamp;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = pos_x;
        odom_tf.transform.translation.y = pos_y;
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(odom_tf);
    }

    // 成员变量
    std::shared_ptr<SportClient> sport_client_;
    bool odom_enabled_;
    bool tf_enabled_;

    // ROS接口
    rclcpp::CallbackGroup::SharedPtr cmd_cb_group_;
    rclcpp::CallbackGroup::SharedPtr tf_cb_group_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr tf_sub_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto sport_node = std::make_shared<SportClientCmdVel>();

    // 使用多线程执行器以支持多核处理，满足规则5
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sport_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
