#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// For real robot
#include "unitree_api/msg/request.hpp"
#include "go2_driver/sport_client.hpp"

using namespace std::chrono_literals;

class SportClientCmdVel : public rclcpp::Node {
public:
    SportClientCmdVel() : Node("sport_client_cmd_vel") {
        // 初始化运动客户端
        sport_client_ = std::make_shared<SportClient>();
        
        // 订阅速度命令
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                handleVelocity(msg);
            });

        // 运动指令发布
        sport_pub_ = create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
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
        
        if (msg->linear.x == 0 && msg->linear.y == 0 && msg->angular.z == 0) {
            stopRobot();
        }
    }

    // 停止机器人
    void stopRobot() {
        unitree_api::msg::Request req;
        sport_client_->StopMove(req);
        sport_pub_->publish(req);
    }

    // 成员变量
    std::shared_ptr<SportClient> sport_client_;
    
    // ROS接口
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr sport_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SportClientCmdVel>());
    rclcpp::shutdown();
    return 0;
}
