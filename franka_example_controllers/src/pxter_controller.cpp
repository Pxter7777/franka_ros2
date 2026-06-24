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

#include "franka_example_controllers/pxter_controller.hpp"

#include <cassert>
#include <cmath>
#include <exception>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration PxterController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration PxterController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type PxterController::update(const rclcpp::Time& /*time*/,
                                                          const rclcpp::Duration& /*period*/) {
  updateJointStates();
  std::lock_guard<std::mutex> lock(goal_mutex_);

  // If there is no active motion, start the next queued goal (if any).
  if (!motion_generator_ && !goal_queue_.empty()) {
    Vector7d next_goal = goal_queue_.front();
    goal_queue_.pop();
    motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, next_goal);
    start_time_ = this->get_node()->now();
    RCLCPP_INFO(get_node()->get_logger(), "Starting new motion from queue.");
  }

  // Decide the position to track this cycle.
  Vector7d q_desired;
  if (motion_generator_) {
    auto trajectory_time = this->get_node()->now() - start_time_;
    auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
    q_desired = motion_generator_output.first;
    if (motion_generator_output.second) {
      // Motion finished: remember this goal as the hold target and clear the
      // generator so the next cycle can pick up a new goal from the queue.
      hold_position_ = q_desired;
      motion_generator_ = nullptr;
      RCLCPP_INFO(get_node()->get_logger(), "Motion finished.");
    }
  } else {
    // No active motion and nothing queued: actively hold the last goal instead
    // of commanding zero torque (which let the arm sag away from the goal).
    q_desired = hold_position_;
  }

  // Always apply the tracking/holding PD law; never go limp.
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  for (int i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn PxterController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<std::vector<double>>("start_joint_configuration",
                                      {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PxterController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  auto start_joint_configuration_vector =
      get_node()->get_parameter("start_joint_configuration").as_double_array();

  Vector7d start_goal;
  Eigen::Map<Eigen::VectorXd>(start_goal.data(), num_joints) =
      Eigen::Map<Eigen::VectorXd>(start_joint_configuration_vector.data(), num_joints);
  goal_queue_.push(start_goal); // Push the initial start position onto the queue

  if (k_gains.empty() || d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "Gains parameters not set");
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

  goal_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/pxter_controller/goal", 10,
      std::bind(&PxterController::goalCallback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn PxterController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  // Hold the current pose until the first queued goal takes over, so an idle
  // cycle never commands zero torque.
  hold_position_ = q_;
  // The first motion will be started by the update loop when it finds the first goal in the queue.
  RCLCPP_INFO(get_node()->get_logger(), "Controller activated. Waiting for goals in the queue.");
  return CallbackReturn::SUCCESS;
}

void PxterController::goalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != num_joints) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received goal with wrong number of joints! Expected %d, got %zu.",
                 num_joints, msg->data.size());
    return;
  }

  std::lock_guard<std::mutex> lock(goal_mutex_);
  Vector7d new_goal;
  Eigen::Map<const Eigen::Matrix<double, 7, 1>> new_goal_map(msg->data.data());
  new_goal = new_goal_map;
  goal_queue_.push(new_goal);
  RCLCPP_INFO(get_node()->get_logger(), "New goal added to the queue.");
}

void PxterController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PxterController,
                       controller_interface::ControllerInterface)