#pragma once

#include <algorithm>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace plan_env {

/**
 * Declare a parameter when it is absent, then return its value.
 *
 * Keeping this helper in plan_env preserves the parameter initialization
 * contract used by the original whole-body planner: parameter overrides are
 * visible even when the owning node did not declare them beforehand.
 */
template <typename T>
T declare_or_get_parameter(const rclcpp::Node::SharedPtr &node,
                           const std::string &name,
                           const T &default_value) {
  if (!node) {
    return default_value;
  }
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

/** Declare a scalar or per-joint double parameter without losing overrides. */
inline void declare_manipulator_joint_limits_param_if_needed(
    const rclcpp::Node::SharedPtr &node, const std::string &param_name,
    const double scalar_default, const int manipulator_dof) {
  if (!node || node->has_parameter(param_name)) {
    return;
  }

  const auto overrides = node->get_node_parameters_interface()
                             ->get_parameter_overrides();
  const auto it = overrides.find(param_name);
  if (it != overrides.end()) {
    const rclcpp::ParameterValue &value = it->second;
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      node->declare_parameter(param_name, value.get<double>());
      return;
    }
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      node->declare_parameter(param_name, value.get<std::vector<double>>());
      return;
    }
    RCLCPP_WARN(node->get_logger(),
                "Parameter %s has an unsupported override type; using a "
                "vector default",
                param_name.c_str());
  }

  const int dof = std::max(manipulator_dof, 1);
  node->declare_parameter(
      param_name, std::vector<double>(static_cast<std::size_t>(dof),
                                      scalar_default));
}

/** Read a scalar or vector joint limit into a fixed-size Eigen vector. */
inline Eigen::VectorXd declare_or_get_manipulator_joint_limits(
    const rclcpp::Node::SharedPtr &node, const std::string &param_name,
    const double scalar_default, const int manipulator_dof) {
  Eigen::VectorXd result =
      Eigen::VectorXd::Constant(std::max(manipulator_dof, 0), scalar_default);
  if (!node || manipulator_dof <= 0) {
    return result;
  }

  declare_manipulator_joint_limits_param_if_needed(
      node, param_name, scalar_default, manipulator_dof);
  const rclcpp::Parameter &parameter = node->get_parameter(param_name);
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      result.setConstant(parameter.as_double());
      break;
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY: {
      const std::vector<double> values = parameter.as_double_array();
      if (static_cast<int>(values.size()) != manipulator_dof) {
        RCLCPP_WARN(node->get_logger(),
                    "Parameter %s expected %d elements, got %zu; using "
                    "scalar fill",
                    param_name.c_str(), manipulator_dof, values.size());
        if (!values.empty()) {
          result.setConstant(values.front());
        }
      } else {
        for (int i = 0; i < manipulator_dof; ++i) {
          result(i) = values[static_cast<std::size_t>(i)];
        }
      }
      break;
    }
    default:
      RCLCPP_WARN(node->get_logger(),
                  "Parameter %s has an unsupported type; using default %.4g",
                  param_name.c_str(), scalar_default);
      break;
  }
  return result;
}

}  // namespace plan_env
