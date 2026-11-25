#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace sonar_processing
{

// Locks the weak node pointer handed to a stage's configure().
inline rclcpp_lifecycle::LifecycleNode::SharedPtr lock_node(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & node, const std::string & stage_id)
{
  auto locked = node.lock();
  if (!locked) {
    throw std::runtime_error("stage '" + stage_id + "': parent node is gone");
  }
  return locked;
}

// Declares `name` with the type of T and NO default value, then reads it. Throws
// std::runtime_error naming the parameter if it is not set (e.g. missing from the YAML).
// The int overload reads PARAMETER_INTEGER; the YAML must then hold an integer (5, not 5.0).
template<typename T>
T declare_and_get(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, rclcpp::ParameterValue(T{}).get_type());
  }
  try {
    return static_cast<T>(node->get_parameter(name).get_value<T>());
  } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
    throw std::runtime_error("missing parameter '" + name + "'");
  }
}

// Same as declare_and_get but with a default value. Only for the few params where "unset" is a
// meaningful value that the YAML cannot spell reliably (empty arrays).
template<typename T>
T declare_and_get_or(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, rclcpp::ParameterValue(default_value));
  }
  return static_cast<T>(node->get_parameter(name).get_value<T>());
}

// Reads "<stage_id>.plugin". No fallback class: a missing key is an error.
inline std::string get_plugin_type(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & stage_id)
{
  const std::string name = stage_id + ".plugin";
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, rclcpp::PARAMETER_STRING);
  }
  try {
    return node->get_parameter(name).as_string();
  } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
    throw std::runtime_error("missing '" + name + "'");
  }
}

}  // namespace sonar_processing
