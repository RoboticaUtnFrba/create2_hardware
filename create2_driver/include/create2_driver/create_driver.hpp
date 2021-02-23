/**
 * \copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.
 * Software License Agreement (BSD)
 * \file      create_driver.h
 * \authors   Jacob Perron <jacobmperron@gmail.com>
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of Autonomy Lab nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once  // NOLINT(build/header_guard)

#include <create/create.h>
#include <tf2_ros/transform_broadcaster.h>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

static const double COVARIANCE[36] = {
  1e-5, 1e-5, 0.0, 0.0, 0.0, 1e-5,  // NOLINT(whitespace/braces)
  1e-5, 1e-5, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0,  1e-5, 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
  1e-5, 0.0,  0.0, 0.0, 0.0, 0.0,  0.0, 1e-5, 0.0,  1e-5, 1e-5, 0.0, 0.0, 0.0, 1e-5};

class CreateDriver : public rclcpp::Node
{
private:
  create::Create * robot_;
  create::RobotModel model_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_joint_pub_;

  rclcpp::TimerBase::SharedPtr loop_timer_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  diagnostic_updater::Updater diagnostics_;

  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::TransformStamped tf_odom_;
  rclcpp::Time last_cmd_vel_time_;
  sensor_msgs::msg::JointState joint_state_msg_;
  bool is_running_slowly_;

  // ROS params
  std::string dev_;
  std::string base_frame_;
  std::string odom_frame_;
  double latch_duration_;
  double loop_hz_;
  bool publish_tf_;
  int baud_;

  void cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr msg);

  bool update();
  void updateBatteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateSafetyDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateModeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void publishOdom();
  void publishJointState();

public:
  CreateDriver();
  ~CreateDriver();
  virtual void spinOnce();
};  // class CreateDriver
