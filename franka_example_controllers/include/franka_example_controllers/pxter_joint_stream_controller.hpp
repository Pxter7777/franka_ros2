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

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ruckig/ruckig.hpp>

#include "std_msgs/msg/float64_multi_array.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/// Streams absolute joint goals to the arm, tracking them continuously.
///
/// Unlike pxter_controller (which runs each queued goal point-to-point, stopping at
/// every waypoint), this controller keeps a single target joint configuration that is
/// REPLACED by the latest message on /pxter_joint_stream_controller/goal (7 absolute
/// joint angles, radians). A Ruckig online trajectory generator drives the commanded
/// joints toward that target jerk-limited, and an impedance (PD + velocity
/// feedforward) law produces the joint torques. The result follows a dense stream of
/// setpoints smoothly instead of stop-starting at each one.
class PxterJointStreamController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  static constexpr size_t kNumJoints = 7;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // Replaces the target with the latest streamed joint configuration.
  void goalCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void updateJointStates();

  std::string arm_id_;
  const int num_joints_ = static_cast<int>(kNumJoints);
  Vector7d q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;

  // Jerk-limited online trajectory generator over the 7 joints.
  std::unique_ptr<ruckig::Ruckig<kNumJoints>> otg_;
  ruckig::InputParameter<kNumJoints> ruckig_input_;
  ruckig::OutputParameter<kNumJoints> ruckig_output_;
  bool trajectory_initialized_{false};

  // Latest target joint configuration (REPLACE semantics). Written by the subscriber
  // callback, read in update(); guarded by goal_mutex_.
  std::mutex goal_mutex_;
  Vector7d target_joints_{Vector7d::Zero()};
  bool target_valid_{false};

  // Control period fed to Ruckig (must match controller_manager update_rate).
  double control_cycle_time_{0.001};
  // Ruckig per-joint kinematic limits (parameters), rad/s, rad/s^2, rad/s^3.
  double max_velocity_{1.0};
  double max_acceleration_{4.0};
  double max_jerk_{40.0};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_subscriber_;
};

}  // namespace franka_example_controllers
