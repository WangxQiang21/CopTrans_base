#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace fwis_ros {

// Launch overrides are auto-declared. Declare defaults for the remaining parameters.
template <typename T>
void parameter(rclcpp::Node& node, const std::string& name, T& value, const T& fallback) {
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, fallback);
  }
  node.get_parameter(name, value);
}

inline rclcpp::QoS map_qos(rclcpp::Node& node) {
  bool transient_local = true;
  parameter(node, "map_transient_local", transient_local, true);
  auto qos = rclcpp::QoS(1);
  if (transient_local) {
    qos.transient_local();
  }
  return qos;
}

}  // namespace fwis_ros
