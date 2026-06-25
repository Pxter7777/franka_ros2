// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "franka_example_controllers/pxter_joint_stream_controller.hpp"

#include <cassert>
#include <exception>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
PxterJointStreamController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
PxterJointStreamController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type PxterJointStreamController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();

  // First cycle: anchor the trajectory generator at the current joints (zero target
  // velocity), and seed the target with the current pose so the arm holds still until
  // a goal arrives.
  if (!trajectory_initialized_) {
    for (size_t i = 0; i < kNumJoints; ++i) {
      ruckig_input_.current_position[i] = q_(static_cast<int>(i));
      ruckig_input_.current_velocity[i] = dq_(static_cast<int>(i));
      ruckig_input_.current_acceleration[i] = 0.0;
      ruckig_input_.target_position[i] = q_(static_cast<int>(i));
      ruckig_input_.target_velocity[i] = 0.0;
      ruckig_input_.target_acceleration[i] = 0.0;
      ruckig_input_.max_velocity[i] = max_velocity_;
      ruckig_input_.max_acceleration[i] = max_acceleration_;
      ruckig_input_.max_jerk[i] = max_jerk_;
    }
    std::lock_guard<std::mutex> lock(goal_mutex_);
    target_joints_ = q_;
    target_valid_ = true;
    trajectory_initialized_ = true;
  }

  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (target_valid_) {
      for (size_t i = 0; i < kNumJoints; ++i) {
        ruckig_input_.target_position[i] = target_joints_(static_cast<int>(i));
      }
    }
  }

  const ruckig::Result result = otg_->update(ruckig_input_, ruckig_output_);
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
    RCLCPP_ERROR(get_node()->get_logger(), "Ruckig trajectory update failed (code %d).",
                 static_cast<int>(result));
    return controller_interface::return_type::ERROR;
  }
  ruckig_output_.pass_to_input(ruckig_input_);

  // Impedance law tracking the Ruckig setpoint, with velocity feedforward so the arm
  // follows a moving target instead of damping to a stop at each waypoint.
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Vector7d tau_d_calculated;
  for (size_t i = 0; i < kNumJoints; ++i) {
    const int j = static_cast<int>(i);
    const double q_desired = ruckig_output_.new_position[i];
    const double dq_desired = ruckig_output_.new_velocity[i];
    tau_d_calculated(j) =
        k_gains_(j) * (q_desired - q_(j)) + d_gains_(j) * (dq_desired - dq_filtered_(j));
  }
  for (size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(static_cast<int>(i)));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn PxterJointStreamController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<double>("control_cycle_time", 0.001);
    auto_declare<double>("max_velocity", 1.0);
    auto_declare<double>("max_acceleration", 4.0);
    auto_declare<double>("max_jerk", 40.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PxterJointStreamController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  control_cycle_time_ = get_node()->get_parameter("control_cycle_time").as_double();
  max_velocity_ = get_node()->get_parameter("max_velocity").as_double();
  max_acceleration_ = get_node()->get_parameter("max_acceleration").as_double();
  max_jerk_ = get_node()->get_parameter("max_jerk").as_double();

  if (k_gains.empty() || d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "Gains parameters not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != kNumJoints || d_gains.size() != kNumJoints) {
    RCLCPP_FATAL(get_node()->get_logger(), "Gains must have %zu entries", kNumJoints);
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints_; ++i) {
    k_gains_(i) = k_gains.at(static_cast<size_t>(i));
    d_gains_(i) = d_gains.at(static_cast<size_t>(i));
  }
  dq_filtered_.setZero();

  goal_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/pxter_joint_stream_controller/goal", 10,
      std::bind(&PxterJointStreamController::goalCallback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn PxterJointStreamController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  otg_ = std::make_unique<ruckig::Ruckig<kNumJoints>>(control_cycle_time_);
  trajectory_initialized_ = false;
  dq_filtered_.setZero();
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    target_valid_ = false;
  }
  RCLCPP_INFO(get_node()->get_logger(),
              "Joint stream controller activated (%.4f s cycle). Holding current pose until a "
              "goal arrives on /pxter_joint_stream_controller/goal.",
              control_cycle_time_);
  return CallbackReturn::SUCCESS;
}

void PxterJointStreamController::goalCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != kNumJoints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Goal needs %zu joints, got %zu.", kNumJoints,
                 msg->data.size());
    return;
  }
  std::lock_guard<std::mutex> lock(goal_mutex_);
  for (size_t i = 0; i < kNumJoints; ++i) {
    target_joints_(static_cast<int>(i)) = msg->data[i];
  }
  target_valid_ = true;
}

void PxterJointStreamController::updateJointStates() {
  for (int i = 0; i < num_joints_; ++i) {
    const auto& position_interface = state_interfaces_.at(static_cast<size_t>(2 * i));
    const auto& velocity_interface = state_interfaces_.at(static_cast<size_t>(2 * i + 1));
    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PxterJointStreamController,
                       controller_interface::ControllerInterface)
