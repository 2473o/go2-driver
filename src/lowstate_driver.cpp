#include "rclcpp/rclcpp.hpp"
#include <unitree_go/msg/low_state.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "go2_driver/msg/leg_sensor.hpp"

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


class LowStateToImuNode : public rclcpp::Node
{
public:
    LowStateToImuNode() : Node("lowstate_driver_node")
    {

        declare_and_get_parameter(this, "imu_enable", imu_enable_, true);
        declare_and_get_parameter(this, "leg_sensor_enable", leg_sensor_enable_, false);
        declare_and_get_parameter(this, "imu_topic", imu_topic_, std::string("/imu"));
        declare_and_get_parameter(this, "leg_sensor_topic", leg_sensor_topic_, std::string("/leg_sensor"));
        // 1. 订阅 LowState 消息
        lowstate_sub_ = this->create_subscription<unitree_go::msg::LowState>(
            "/lowstate", rclcpp::SensorDataQoS(), 
            std::bind(&LowStateToImuNode::lowstate_callback, this, std::placeholders::_1));
        
        // 2. 发布标准 Imu 话题
        if (imu_enable_) {
            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
        }
        
        if (leg_sensor_enable_) {
            // 3. 发布 LegSensor 话题
            leg_sensor_pub_ = this->create_publisher<go2_driver::msg::LegSensor>(leg_sensor_topic_, 10);
        }
    }

private:
    // 低层状态信息订阅
    rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
    // IMU 信息发布
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    // LegSensor 信息发布
    bool imu_enable_;
    bool leg_sensor_enable_;
    std::string imu_topic_;
    std::string leg_sensor_topic_;
    rclcpp::Publisher<go2_driver::msg::LegSensor>::SharedPtr leg_sensor_pub_;

    // 从低层状态信息中提取IMU信息
    void lowstate_callback(const unitree_go::msg::LowState::SharedPtr lowstate_msg)
    {
        auto current_time = this->now();

        if (imu_enable_) {
            // 创建 IMU 消息
            auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
            imu_msg->header.stamp = current_time;
            imu_msg->header.frame_id = "imu_link"; 

            // 四元数转换（Unitree 格式 [w,x,y,z] → ROS [x,y,z,w]）
            const auto& quat = lowstate_msg->imu_state.quaternion;
            imu_msg->orientation.x = quat[1];  
            imu_msg->orientation.y = quat[2];  
            imu_msg->orientation.z = quat[3];  
            imu_msg->orientation.w = quat[0];  
            
            // 角速度（单位：rad/s）
            imu_msg->angular_velocity.x = lowstate_msg->imu_state.gyroscope[0];
            imu_msg->angular_velocity.y = lowstate_msg->imu_state.gyroscope[1];
            imu_msg->angular_velocity.z = lowstate_msg->imu_state.gyroscope[2];
            
            // 加速度
            imu_msg->linear_acceleration.x = lowstate_msg->imu_state.accelerometer[0];
            imu_msg->linear_acceleration.y = lowstate_msg->imu_state.accelerometer[1];
            imu_msg->linear_acceleration.z = lowstate_msg->imu_state.accelerometer[2];

            // 协方差矩阵
            // 角速度协方差
            imu_msg->angular_velocity_covariance[0] = 0.0002;  // xx
            imu_msg->angular_velocity_covariance[4] = 0.0002;  // yy
            imu_msg->angular_velocity_covariance[8] = 0.0002;  // zz
            
            // 线性加速度协方差
            imu_msg->linear_acceleration_covariance[0] = 0.02;  // xx
            imu_msg->linear_acceleration_covariance[4] = 0.02;  // yy
            imu_msg->linear_acceleration_covariance[8] = 0.02;  // zz
            
            // 方向协方差（-1表示未知）
            imu_msg->orientation_covariance[0] = -1.0;

            // 发布 IMU 消息
            imu_pub_->publish(std::move(imu_msg));
        }

        if (leg_sensor_enable_) {
            // 发布 LegSensor 消息
            auto leg_msg = std::make_unique<go2_driver::msg::LegSensor>();
            leg_msg->header.stamp = current_time;
            leg_msg->header.frame_id = "base_link";

            if (lowstate_msg->motor_state.size() >= 12) {
                for(int i=0; i<12; ++i) {
                    leg_msg->q[i] = lowstate_msg->motor_state[i].q;
                    leg_msg->dq[i] = lowstate_msg->motor_state[i].dq;
                    leg_msg->tau[i] = lowstate_msg->motor_state[i].tau_est;
                }
            }

            if (lowstate_msg->foot_force.size() >= 4) {
                for(int i=0; i<4; ++i) {
                    leg_msg->foot_force[i] = (float)lowstate_msg->foot_force[i];
                }
            }

            leg_msg->imu_state = lowstate_msg->imu_state;

            leg_sensor_pub_->publish(std::move(leg_msg));
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LowStateToImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
