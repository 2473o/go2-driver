#ifndef GO2_DRIVER_UTILITIES_HPP_
#define GO2_DRIVER_UTILITIES_HPP_

#include <string>
#include <rclcpp/rclcpp.hpp>

template<typename T>
void declare_and_get_parameter(rclcpp::Node* node, std::string name, T& variable, const T& default_value) {
    node->declare_parameter<T>(name, default_value);
    node->get_parameter(name, variable);
}

#endif
