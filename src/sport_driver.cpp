#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

// For real robot
#include "unitree_api/msg/request.hpp"
#include "go2_driver/sport_client.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

using namespace std::chrono_literals;

class SportClientCmdVel : public rclcpp::Node {
public:
    SportClientCmdVel() : Node("sport_client_driver"), body_height_(0.0) {
        // 初始化运动客户端
        sport_client_ = std::make_shared<SportClient>();

        // 创建互斥回调组，支持多线程、多核处理，满足规则5
        cmd_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        tf_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions cmd_sub_opts;
        cmd_sub_opts.callback_group = cmd_cb_group_;

        rclcpp::SubscriptionOptions tf_sub_opts;
        tf_sub_opts.callback_group = tf_cb_group_;

        // 订阅速度命令
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", rclcpp::QoS(10),
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                handleVelocity(msg);
            }, cmd_sub_opts);

        // 运动指令发布
        sport_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", rclcpp::QoS(10));

        // 动态TF订阅与定时器 (合并原 TFDynamicBroadcaster 逻辑)
        tf_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", rclcpp::QoS(10), 
            std::bind(&SportClientCmdVel::state_cb, this, std::placeholders::_1), tf_sub_opts);
        
        tf_timer_ = this->create_wall_timer(
            100ms, std::bind(&SportClientCmdVel::timer_cb, this), timer_cb_group_);
            
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    // 处理速度命令
    void handleVelocity(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 直接转发速度命令
        unitree_api::msg::Request req;
        sport_client_->Move(req, 
                          msg->linear.x,
                          msg->linear.y,
                          msg->angular.z);
        sport_pub_->publish(req);
    }

    // 处理机器人状态信息，更新高度
    void state_cb(const unitree_go::msg::SportModeState::SharedPtr state_msg) {
        body_height_ = state_msg->body_height + 0.057;
    }

    // 定时发布动态TF
    void timer_cb() {
        geometry_msgs::msg::TransformStamped transform_;
        transform_.header.stamp = this->get_clock()->now();
        transform_.header.frame_id = "base_footprint";
        transform_.child_frame_id = "base_link";
        transform_.transform.translation.x = 0.0;
        transform_.transform.translation.y = 0.0;
        transform_.transform.translation.z = body_height_;
        
        tf2::Quaternion qtn;
        qtn.setRPY(0.0, 0.0, 0.0);
        transform_.transform.rotation.x = qtn.x();
        transform_.transform.rotation.y = qtn.y();
        transform_.transform.rotation.z = qtn.z();
        transform_.transform.rotation.w = qtn.w();
        
        tf_broadcaster_->sendTransform(transform_);
    }

    // 成员变量
    std::shared_ptr<SportClient> sport_client_;
    double body_height_;
    
    // ROS接口
    rclcpp::CallbackGroup::SharedPtr cmd_cb_group_;
    rclcpp::CallbackGroup::SharedPtr tf_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr tf_sub_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto sport_node = std::make_shared<SportClientCmdVel>();
    
    // 使用多线程执行器以支持多核处理，满足规则5
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sport_node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
