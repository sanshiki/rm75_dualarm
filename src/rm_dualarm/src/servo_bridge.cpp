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

#include "rm_dualarm/servo_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace rm_dualarm
{

ServoBridge::ServoBridge(const rclcpp::NodeOptions & options)
: Node("servo_bridge", options),
  stopped_(false)
{
  // ---- Declare and read parameters ----
  declare_parameter("arm_dof", 7);
  declare_parameter("follow_mode", true);
  declare_parameter("publish_rate", 50.0);
  declare_parameter("command_timeout", 0.15);   // 150 ms
  declare_parameter("halt_on_timeout", true);
  declare_parameter("servo_input_topic", "/servo_bridge/joint_trajectory_in");
  declare_parameter("driver_topic", "/rm_driver/movej_canfd_cmd");

  arm_dof_ = get_parameter("arm_dof").as_int();
  follow_mode_ = get_parameter("follow_mode").as_bool();
  publish_rate_ = get_parameter("publish_rate").as_double();
  command_timeout_ = get_parameter("command_timeout").as_double();
  halt_on_timeout_ = get_parameter("halt_on_timeout").as_bool();

  std::string servo_input = get_parameter("servo_input_topic").as_string();
  std::string driver_topic = get_parameter("driver_topic").as_string();

  // Clamp DOF to valid range
  if (arm_dof_ < 6) {
    arm_dof_ = 6;
  } else if (arm_dof_ > 7) {
    arm_dof_ = 7;
  }

  RCLCPP_INFO(
    get_logger(),
    "ServoBridge started: dof=%d, rate=%.1f Hz, timeout=%.3f s, "
    "servo_in=%s, driver_out=%s",
    arm_dof_, publish_rate_, command_timeout_,
    servo_input.c_str(), driver_topic.c_str());

  // ---- Subscriptions ----
  servo_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
    servo_input, rclcpp::SensorDataQoS(),
    std::bind(&ServoBridge::servoCallback, this, std::placeholders::_1));

  stop_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/servo_bridge/stop", rclcpp::QoS(1),
    std::bind(&ServoBridge::stopCallback, this, std::placeholders::_1));

  // ---- Publisher ----
  jointpos_pub_ = create_publisher<rm_ros_interfaces::msg::Jointpos>(
    driver_topic, rclcpp::SensorDataQoS());

  // ---- Timer ----
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ServoBridge::timerCallback, this));

  last_command_time_ = now();
}

// ------------------------------------------------------------------
void ServoBridge::servoCallback(
  const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (stopped_) {
    return;   // Emergency-stop active — ignore incoming commands
  }

  std::lock_guard<std::mutex> lock(mutex_);
  latest_trajectory_ = msg;
  last_command_time_ = now();
}

// ------------------------------------------------------------------
void ServoBridge::stopCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  RCLCPP_WARN(get_logger(), "ServoBridge STOP received — halting");
  stopped_ = true;
}

// ------------------------------------------------------------------
void ServoBridge::timerCallback()
{
  trajectory_msgs::msg::JointTrajectory::SharedPtr traj;
  bool stale = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Check timeout
    double elapsed = (now() - last_command_time_).seconds();
    if (elapsed > command_timeout_) {
      stale = true;
    }
    traj = latest_trajectory_;   // may be nullptr if never received
    // Un-pause if new command arrives after a stop
    if (elapsed < command_timeout_) {
      stopped_ = false;
    }
  }

  // Build and publish Jointpos message
  rm_ros_interfaces::msg::Jointpos jp;
  jp.follow = follow_mode_;
  jp.dof = static_cast<uint8_t>(arm_dof_);
  jp.expand = 0.0f;

  if (stopped_) {
    // Publish halt message with zero velocity (empty joints = hold)
    // rm_driver interprets empty joint array as "no motion"
    jp.joint.resize(arm_dof_, 0.0f);
    jointpos_pub_->publish(jp);
    return;
  }

  if (stale && halt_on_timeout_) {
    // No recent command — stop sending or send last known position
    if (traj && !traj->points.empty()) {
      auto & last_pt = traj->points.back();
      jp.joint.assign(last_pt.positions.begin(), last_pt.positions.end());
    } else {
      jp.joint.resize(arm_dof_, 0.0f);
    }
    jointpos_pub_->publish(jp);
    return;
  }

  if (!traj || traj->points.empty()) {
    // No data yet — nothing to publish
    return;
  }

  // Use the last point in the trajectory as the target.
  // MoveIt2 Servo typically publishes single-point trajectories at high rate
  // (50 Hz), so simple "send the latest" works well without interpolation.
  // When Servo does publish multi-point mini-trajectories, we send the last
  // point to minimise latency.
  auto & pt = traj->points.back();
  jp.joint.assign(pt.positions.begin(), pt.positions.end());

  // Truncate / pad to match arm_dof_
  if (jp.joint.size() > static_cast<size_t>(arm_dof_)) {
    jp.joint.resize(arm_dof_);
  } else if (jp.joint.size() < static_cast<size_t>(arm_dof_)) {
    // Pad with zeros — should not normally happen
    jp.joint.resize(arm_dof_, 0.0f);
  }

  jointpos_pub_->publish(jp);
}

// ------------------------------------------------------------------
// Static helpers
// ------------------------------------------------------------------
std::vector<double> ServoBridge::lerpJoints(
  const std::vector<double> & from,
  const std::vector<double> & to,
  double alpha)
{
  size_t n = std::max(from.size(), to.size());
  std::vector<double> result(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    double f = (i < from.size()) ? from[i] : 0.0;
    double t = (i < to.size())   ? to[i]   : 0.0;
    result[i] = f + alpha * (t - f);
  }
  return result;
}

rm_ros_interfaces::msg::Jointpos ServoBridge::makeJointpos(
  const std::vector<double> & joints)
{
  rm_ros_interfaces::msg::Jointpos jp;
  jp.joint.assign(joints.begin(), joints.end());
  jp.follow = follow_mode_;
  jp.dof = static_cast<uint8_t>(joints.size());
  jp.expand = 0.0f;
  return jp;
}

}  // namespace rm_dualarm

// ---- main entry point ----
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rm_dualarm::ServoBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
