#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class LowStateToImuNode : public rclcpp::Node
{
public:
    LowStateToImuNode() : Node("sportstate_to_imu_node")
    {
        // 1. 订阅 LowState 消息
        sportstate_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", rclcpp::SensorDataQoS(), 
            std::bind(&LowStateToImuNode::sportstate_callback, this, std::placeholders::_1));
        
        // 2. 发布标准 Imu 话题
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        
        // 3. 创建 TF 发布器
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        RCLCPP_INFO(this->get_logger(), "IMU 转换节点已启动");
    }

private:
    // 低层状态信息订阅
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sportstate_sub_;
    // IMU 信息发布
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    // TF 广播器
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 从低层状态信息中提取IMU信息
    void sportstate_callback(const unitree_go::msg::SportModeState::SharedPtr sportstate_smg)
    {
        // 创建 IMU 消息
        auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
        
        imu_msg->header.stamp = this->now();
        // imu_msg->header.stamp.sec = sportstate_smg->stamp.sec;
        // imu_msg->header.stamp.nanosec = sportstate_smg->stamp.nanosec;
        imu_msg->header.frame_id = "imu_link";

        // 四元数转换（Unitree 格式 [w,x,y,z] → ROS [x,y,z,w]）
        const auto& quat = sportstate_smg->imu_state.quaternion;
        imu_msg->orientation.x = quat[1];  
        imu_msg->orientation.y = quat[2];  
        imu_msg->orientation.z = quat[3];  
        imu_msg->orientation.w = quat[0];  

        // 角速度（单位：rad/s）
        imu_msg->angular_velocity.x = sportstate_smg->imu_state.gyroscope[0];
        imu_msg->angular_velocity.y = sportstate_smg->imu_state.gyroscope[1];
        imu_msg->angular_velocity.z = sportstate_smg->imu_state.gyroscope[2];
        
        // 加速度
        imu_msg->linear_acceleration.x = sportstate_smg->imu_state.accelerometer[0];
        imu_msg->linear_acceleration.y = sportstate_smg->imu_state.accelerometer[1];
        imu_msg->linear_acceleration.z = sportstate_smg->imu_state.accelerometer[2];

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

        // 发布 TF 变换
        // publish_tf_transform();
    }

    // 发布 TF 变换
    void publish_tf_transform()
    {
        geometry_msgs::msg::TransformStamped tf;

        // base_link -> imu_link
        tf.header.stamp = this->now();
        tf.header.frame_id = "base_link";
        tf.child_frame_id = "imu_link";

        tf.transform.translation.x = -0.02557;  // 前向偏移2.557cm
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.04232;   // 向上偏移4.232cm

        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tf);

        /*
        // imu_link -> rslidar
        geometry_msgs::msg::TransformStamped lidar_tf;
        lidar_tf.header.stamp = this->now();
        lidar_tf.header.frame_id = "imu_link";
        lidar_tf.child_frame_id = "rslidar";

        lidar_tf.transform.translation.x = 0.1710;  // 平移参数
        lidar_tf.transform.translation.y = 0.0;
        lidar_tf.transform.translation.z = 0.0908;

        lidar_tf.transform.rotation.x = 0.0;  // 旋转矩阵对应的四元数
        lidar_tf.transform.rotation.y = 0.0;
        lidar_tf.transform.rotation.z = 0.0;
        lidar_tf.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(lidar_tf);
        */
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