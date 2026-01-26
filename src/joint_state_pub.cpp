#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::placeholders;

template<typename T>
/**
 * @brief Declare and get a parameter from the ROS2 node.
 * @param node Shared pointer to the ROS2 node.
 * @param name Name of the parameter.
 * @param variable Reference to the variable where the parameter value will be stored.
 * @param default_value Default value to use if the parameter is not set.
 */
void declare_and_get_parameter(rclcpp::Node::SharedPtr node, std::string name, T& variable, const T& default_value) {
    node->declare_parameter<T>(name, default_value);
    node->get_parameter(name, variable);
}

class JointStatePublisherNode : public rclcpp::Node
{
public:
    JointStatePublisherNode() : Node("joint_state_publisher")
    {
        RCLCPP_INFO(this->get_logger(), "关节状态发布节点启动");

        // 创建关节状态发布者
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // 订阅低层状态话题
        low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", 10, std::bind(&JointStatePublisherNode::low_state_cb, this, _1));
        }

private:
    // 发布者&订阅者
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

    // 低层状态回调：解析关节数据并发布
    void low_state_cb(const unitree_go::msg::LowState::SharedPtr low_state)
    { 
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = this->now();          
        joint_state.name = {
            "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint", 
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
            "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint", 
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint" 
        };

        // 填充关节位置数据
        for(size_t i = 0; i < 12; i++)
        {
            auto motor = low_state->motor_state[i];
            joint_state.position.push_back(motor.q);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePublisherNode>());
    rclcpp::shutdown();
    return 0;
}