// Copyright (c) 2024  RM DualArm Contributors
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

#ifndef RM_DUALARM__SERVO_BRIDGE_HPP_
#define RM_DUALARM__SERVO_BRIDGE_HPP_

#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "rm_ros_interfaces/msg/jointpos.hpp"
#include "std_msgs/msg/empty.hpp"

namespace rm_dualarm
{

/// @brief Bridges MoveIt2 Servo JointTrajectory output to rm_driver CANFD
///        streaming topics at high frequency (50 Hz).
///
/// Subscribes to a JointTrajectory topic published by MoveIt2 Servo (or
/// servo_pose_tracking_demo) and re-publishes the joint positions as
/// rm_ros_interfaces::msg::Jointpos messages to the rm_driver CANFD
/// topic at a configurable rate.  This enables real-time servoing on
/// the RealMan arm hardware.
class ServoBridge : public rclcpp::Node
{
public:
  /// @param options Node options forwarded to rclcpp::Node.
  explicit ServoBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- Parameters ----
  int arm_dof_;                  // 6 or 7
  bool follow_mode_;             // true = high-follow streaming
  double publish_rate_;          // Hz (default 50.0)
  double command_timeout_;       // seconds before considering command stale
  bool halt_on_timeout_;         // stop sending if command goes stale

  // ---- Publishers ----
  rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr jointpos_pub_;

  // ---- Subscriptions ----
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr servo_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;

  // ---- Timer ----
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // ---- State ----
  trajectory_msgs::msg::JointTrajectory::SharedPtr latest_trajectory_;
  rclcpp::Time last_command_time_;
  bool stopped_;
  mutable std::mutex mutex_;

  // ---- Callbacks ----
  void servoCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
  void stopCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void timerCallback();

  /// Helper: linear interpolation between two joint position vectors.
  static std::vector<double> lerpJoints(
    const std::vector<double> & from,
    const std::vector<double> & to,
    double alpha);

  /// Helper: fill a Jointpos message from a vector of joint angles.
  rm_ros_interfaces::msg::Jointpos makeJointpos(const std::vector<double> & joints);
};

}  // namespace rm_dualarm

#endif  // RM_DUALARM__SERVO_BRIDGE_HPP_
