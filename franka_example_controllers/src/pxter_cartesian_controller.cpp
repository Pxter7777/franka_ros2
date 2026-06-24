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

#include "franka_example_controllers/pxter_cartesian_controller.hpp"

#include <array>
#include <cmath>
#include <exception>

#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/robot_utils.hpp>

namespace franka_example_controllers {

namespace {

// Rotation vector (axis * angle) of a relative rotation quaternion.
Eigen::Vector3d toRotationVector(const Eigen::Quaterniond& q_rel) {
  const Eigen::AngleAxisd angle_axis(q_rel);
  return angle_axis.angle() * angle_axis.axis();
}

// Inverse of toRotationVector().
Eigen::Quaterniond fromRotationVector(const Eigen::Vector3d& rotation_vector) {
  const double angle = rotation_vector.norm();
  if (angle < 1e-9) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

}  // namespace

controller_interface::InterfaceConfiguration
PxterCartesianController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();
  return config;
}

controller_interface::InterfaceConfiguration
PxterCartesianController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  return config;
}

controller_interface::return_type PxterCartesianController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  auto [current_orientation, current_position] =
      franka_cartesian_pose_->getCurrentOrientationAndTranslation();

  // First cycle: anchor everything to the current pose with zero velocity, so motion
  // starts smoothly from where the robot actually is.
  if (!pose_initialized_) {
    reference_orientation_ = current_orientation;
    ruckig_input_.current_position = {current_position.x(), current_position.y(),
                                      current_position.z(), 0.0, 0.0, 0.0};
    ruckig_input_.current_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ruckig_input_.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ruckig_input_.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ruckig_input_.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ruckig_input_.max_velocity = {max_linear_velocity_, max_linear_velocity_, max_linear_velocity_,
                                  max_angular_velocity_, max_angular_velocity_,
                                  max_angular_velocity_};
    ruckig_input_.max_acceleration = {
        max_linear_acceleration_, max_linear_acceleration_, max_linear_acceleration_,
        max_angular_acceleration_, max_angular_acceleration_, max_angular_acceleration_};
    ruckig_input_.max_jerk = {max_linear_jerk_,  max_linear_jerk_,  max_linear_jerk_,
                              max_angular_jerk_, max_angular_jerk_, max_angular_jerk_};

    std::lock_guard<std::mutex> lock(goal_mutex_);
    target_position_ = current_position;
    target_orientation_ = current_orientation;
    pose_initialized_ = true;
  }

  // Latest target, converted into the Ruckig coordinate vector.
  Eigen::Vector3d target_position;
  Eigen::Quaterniond target_orientation;
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    target_position = target_position_;
    target_orientation = target_orientation_;
  }
  const Eigen::Vector3d target_rotation_vector =
      toRotationVector(reference_orientation_.conjugate() * target_orientation);
  ruckig_input_.target_position = {target_position.x(),        target_position.y(),
                                   target_position.z(),        target_rotation_vector.x(),
                                   target_rotation_vector.y(), target_rotation_vector.z()};

  const ruckig::Result result = otg_->update(ruckig_input_, ruckig_output_);
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
    RCLCPP_ERROR(get_node()->get_logger(), "Ruckig trajectory update failed (code %d).",
                 static_cast<int>(result));
    return controller_interface::return_type::ERROR;
  }
  ruckig_output_.pass_to_input(ruckig_input_);

  const auto& commanded = ruckig_output_.new_position;
  const Eigen::Vector3d commanded_position(commanded[0], commanded[1], commanded[2]);
  Eigen::Quaterniond commanded_orientation =
      reference_orientation_ *
      fromRotationVector(Eigen::Vector3d(commanded[3], commanded[4], commanded[5]));
  commanded_orientation.normalize();

  if (franka_cartesian_pose_->setCommand(commanded_orientation, commanded_position)) {
    return controller_interface::return_type::OK;
  }
  RCLCPP_FATAL(get_node()->get_logger(),
               "Set command failed. Did you activate the elbow command interface?");
  return controller_interface::return_type::ERROR;
}

CallbackReturn PxterCartesianController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          k_elbow_activated_);
  try {
    auto_declare<double>("control_cycle_time", 0.001);
    auto_declare<double>("max_linear_velocity", 0.05);
    auto_declare<double>("max_linear_acceleration", 0.5);
    auto_declare<double>("max_linear_jerk", 5.0);
    auto_declare<double>("max_angular_velocity", 0.3);
    auto_declare<double>("max_angular_acceleration", 2.0);
    auto_declare<double>("max_angular_jerk", 20.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn PxterCartesianController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  control_cycle_time_ = get_node()->get_parameter("control_cycle_time").as_double();
  max_linear_velocity_ = get_node()->get_parameter("max_linear_velocity").as_double();
  max_linear_acceleration_ = get_node()->get_parameter("max_linear_acceleration").as_double();
  max_linear_jerk_ = get_node()->get_parameter("max_linear_jerk").as_double();
  max_angular_velocity_ = get_node()->get_parameter("max_angular_velocity").as_double();
  max_angular_acceleration_ = get_node()->get_parameter("max_angular_acceleration").as_double();
  max_angular_jerk_ = get_node()->get_parameter("max_angular_jerk").as_double();

  // Relax the robot's reflex thresholds to the library defaults, matching the other
  // Cartesian example controllers, so light contact does not abort the motion.
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);
  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");

  goal_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/pxter_cartesian_controller/goal", 10,
      std::bind(&PxterCartesianController::goalCallback, this, std::placeholders::_1));
  RCLCPP_INFO(get_node()->get_logger(),
              "Listening for 6D pose deltas on /pxter_cartesian_controller/goal "
              "(limits: %.3f m/s, %.3f rad/s, jerk-limited via Ruckig).",
              max_linear_velocity_, max_angular_velocity_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn PxterCartesianController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  otg_ = std::make_unique<ruckig::Ruckig<kCartesianDofs>>(control_cycle_time_);
  pose_initialized_ = false;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  RCLCPP_INFO(get_node()->get_logger(), "Controller activated (%.4f s cycle). Holding start pose.",
              control_cycle_time_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn PxterCartesianController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void PxterCartesianController::goalCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  constexpr size_t kGoalSize = 6;
  if (msg->data.size() != kGoalSize) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Goal needs %zu values [dx,dy,dz,droll,dpitch,dyaw], got %zu.", kGoalSize,
                 msg->data.size());
    return;
  }

  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (!pose_initialized_) {
    RCLCPP_WARN(get_node()->get_logger(), "Current pose not initialized yet; dropping goal.");
    return;
  }

  target_position_ += Eigen::Vector3d(msg->data[0], msg->data[1], msg->data[2]);
  const Eigen::Quaterniond delta_rotation =
      Eigen::AngleAxisd(msg->data[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(msg->data[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(msg->data[3], Eigen::Vector3d::UnitX());
  target_orientation_ = delta_rotation * target_orientation_;
  target_orientation_.normalize();
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::PxterCartesianController,
                       controller_interface::ControllerInterface)
