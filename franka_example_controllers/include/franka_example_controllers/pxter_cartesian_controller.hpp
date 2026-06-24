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

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ruckig/ruckig.hpp>

#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/// Streams end-effector pose goals to the robot's native Cartesian-pose interface.
///
/// Incoming messages on /pxter_cartesian_controller/goal are 6-element pose DELTAS
/// [dx, dy, dz, droll, dpitch, dyaw] in the base frame (translation in metres,
/// rotation as extrinsic XYZ-Euler radians). Each delta is accumulated onto a target
/// pose. A Ruckig online trajectory generator then produces a jerk-limited command
/// stream toward that target, which the robot's Cartesian motion generator requires
/// (it aborts on velocity/acceleration discontinuities). The robot firmware performs
/// the inverse kinematics, so this controller needs no external IK service.
///
/// Orientation is tracked as a rotation vector relative to the pose held at activation,
/// so it stays well clear of the +/-pi wrap-around for normal streaming motions.
class PxterCartesianController : public controller_interface::ControllerInterface {
 public:
  static constexpr size_t kCartesianDofs = 6;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // Accumulates an incoming pose delta onto the target pose.
  void goalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;
  static constexpr bool k_elbow_activated_{false};

  // Jerk-limited online trajectory generator over [x, y, z, rx, ry, rz], where the
  // last three are the orientation rotation vector relative to reference_orientation_.
  std::unique_ptr<ruckig::Ruckig<kCartesianDofs>> otg_;
  ruckig::InputParameter<kCartesianDofs> ruckig_input_;
  ruckig::OutputParameter<kCartesianDofs> ruckig_output_;

  bool pose_initialized_{false};
  Eigen::Quaterniond reference_orientation_{Eigen::Quaterniond::Identity()};

  // Target pose accumulated from incoming deltas. Written by the (non-realtime)
  // subscriber callback and read in update(); guarded by goal_mutex_.
  std::mutex goal_mutex_;
  Eigen::Vector3d target_position_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond target_orientation_{Eigen::Quaterniond::Identity()};

  // Control loop period fed to Ruckig. Must match controller_manager's update_rate
  // (1000 Hz -> 0.001 s); a wrong value makes the trajectory run too fast/slow.
  double control_cycle_time_{0.001};

  // Ruckig kinematic limits (parameters). Linear in m/s, m/s^2, m/s^3; angular in rad/...
  double max_linear_velocity_{0.05};
  double max_linear_acceleration_{0.5};
  double max_linear_jerk_{5.0};
  double max_angular_velocity_{0.3};
  double max_angular_acceleration_{2.0};
  double max_angular_jerk_{20.0};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_subscriber_;
};

}  // namespace franka_example_controllers
