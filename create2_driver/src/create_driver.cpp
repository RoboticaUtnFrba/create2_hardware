/**
Software License Agreement (BSD)
\file      create_driver.cpp
\authors   Jacob Perron <jacobmperron@gmail.com>
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <create2_driver/create_driver.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <string>


CreateDriver::CreateDriver()
  : Node("create_driver"),
    model_(create::RobotModel::CREATE_2),
    tf_broadcaster_(this),
    diagnostics_(this),
    last_cmd_vel_time_(0),
    is_running_slowly_(false)
{
  dev_ = declare_parameter<std::string>("dev", "/dev/ttyUSB0");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  latch_duration_ = declare_parameter<double>("latch_cmd_duration", 0.2);
  loop_hz_ = declare_parameter<double>("loop_hz", 10.0);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);

  model_ = create::RobotModel::CREATE_2;

  baud_ = declare_parameter<int>("baud", model_.getBaud());

  robot_ = new create::Create(model_);

  if (!robot_->connect(dev_, baud_))
  {
    RCLCPP_FATAL(get_logger(), "[CREATE] Failed to establish serial connection with Create.");
    rclcpp::shutdown();
  }

  RCLCPP_INFO(this->get_logger(), "[CREATE] Connection established.");

  // Start in full control mode
  robot_->setMode(create::MODE_FULL);

  // Show robot's battery level
  RCLCPP_INFO(get_logger(), "[CREATE] Battery level %.2f %%", (robot_->getBatteryCharge() / robot_->getBatteryCapacity()) * 100.0);

  // Set frame_id's
  tf_odom_.header.frame_id = odom_frame_;
  tf_odom_.child_frame_id = base_frame_;
  odom_msg_.header.frame_id = odom_frame_;
  odom_msg_.child_frame_id = base_frame_;
  joint_state_msg_.name.resize(2);
  joint_state_msg_.position.resize(2);
  joint_state_msg_.velocity.resize(2);
  joint_state_msg_.effort.resize(2);
  joint_state_msg_.name[0] = "left_wheel_joint";
  joint_state_msg_.name[1] = "right_wheel_joint";

  // Populate intial covariances
  for (int i = 0; i < 36; i++)
  {
    odom_msg_.pose.covariance[i] = COVARIANCE[i];
    odom_msg_.twist.covariance[i] = COVARIANCE[i];
  }

  // Setup subscribers
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&CreateDriver::cmdVelCallback, this, std::placeholders::_1));
  
  // Setup publishers
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  wheel_joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

  // Setup diagnostics
  diagnostics_.add("Battery Status", this, &CreateDriver::updateBatteryDiagnostics);
  diagnostics_.add("Safety Status", this, &CreateDriver::updateSafetyDiagnostics);
  diagnostics_.add("Serial Status", this, &CreateDriver::updateSerialDiagnostics);
  diagnostics_.add("Base Mode", this, &CreateDriver::updateModeDiagnostics);
  diagnostics_.add("Driver Status", this, &CreateDriver::updateDriverDiagnostics);

  diagnostics_.setHardwareID("iRobot Create 2");

  // Setup update loop
  const auto loop_period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / loop_hz_));
  loop_timer_ = create_wall_timer(loop_period, std::bind(&CreateDriver::spinOnce, this));

  RCLCPP_INFO(get_logger(), "[CREATE] Ready.");
}

CreateDriver::~CreateDriver()
{
  RCLCPP_INFO(get_logger(), "[CREATE] Destruct sequence initiated.");
  robot_->disconnect();
  delete robot_;
}

void CreateDriver::cmdVelCallback(geometry_msgs::msg::Twist::UniquePtr msg)
{
  robot_->drive(msg->linear.x, msg->angular.z);
  last_cmd_vel_time_ = now();
}

bool CreateDriver::update()
{
  publishOdom();
  publishJointState();

  // If last velocity command was sent longer than latch duration, stop robot
  if (last_cmd_vel_time_.nanoseconds() == 0 || now() - last_cmd_vel_time_ >= rclcpp::Duration::from_seconds(latch_duration_))
  {
    robot_->drive(0, 0);
  }

  return true;
}

void CreateDriver::updateBatteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const float charge = robot_->getBatteryCharge();
  const float capacity = robot_->getBatteryCapacity();
  const create::ChargingState charging_state = robot_->getChargingState();
  const float charge_ratio = charge / capacity;

  if (charging_state == create::CHARGE_FAULT)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Charging fault reported by base");
  }
  else if (charge_ratio == 0)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Battery reports no charge");
  }
  else if (charge_ratio < 0.1)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Battery reports less than 10% charge");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Battery is OK");
  }

  stat.add("Charge (Ah)", charge);
  stat.add("Capacity (Ah)", capacity);
  stat.add("Temperature (Celsius)", robot_->getTemperature());
  stat.add("Current (A)", robot_->getCurrent());
  stat.add("Voltage (V)", robot_->getVoltage());

  switch (charging_state)
  {
    case create::CHARGE_NONE:
      stat.add("Charging state", "Not charging");
      break;
    case create::CHARGE_RECONDITION:
      stat.add("Charging state", "Reconditioning");
      break;
    case create::CHARGE_FULL:
      stat.add("Charging state", "Full charge");
      break;
    case create::CHARGE_TRICKLE:
      stat.add("Charging state", "Trickle charging");
      break;
    case create::CHARGE_WAITING:
      stat.add("Charging state", "Waiting");
      break;
    case create::CHARGE_FAULT:
      stat.add("Charging state", "Fault");
      break;
  }
}

void CreateDriver::updateSafetyDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const bool is_wheeldrop = robot_->isWheeldrop();
  const bool is_cliff = robot_->isCliff();
  if (is_wheeldrop)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Wheeldrop detected");
  }
  else if (is_cliff)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Cliff detected");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "No safety issues detected");
  }

  stat.add("Wheeldrop", is_wheeldrop);
  stat.add("Cliff", is_cliff);
}

void CreateDriver::updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const bool is_connected = robot_->connected();
  const uint64_t corrupt_packets = robot_->getNumCorruptPackets();
  const uint64_t total_packets = robot_->getTotalPackets();

  if (!is_connected)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Serial port to base not open");
  }
  else if (corrupt_packets)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "Corrupt packets detected. If the number of corrupt packets is increasing, data may be unreliable");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Serial connection is good");
  }

  stat.add("Corrupt packets", corrupt_packets);
  stat.add("Total packets", total_packets);
}

void CreateDriver::updateModeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const create::CreateMode mode = robot_->getMode();
  switch (mode)
  {
    case create::MODE_UNAVAILABLE:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unknown mode of operation");
      break;
    case create::MODE_OFF:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Mode is set to OFF");
      break;
    case create::MODE_PASSIVE:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Mode is set to PASSIVE");
      break;
    case create::MODE_SAFE:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Mode is set to SAFE");
      break;
    case create::MODE_FULL:
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Mode is set to FULL");
      break;
  }
}

void CreateDriver::updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (is_running_slowly_)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Internal loop running slowly");
  }
  else
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Maintaining loop frequency");
  }
}

void CreateDriver::publishOdom()
{
  create::Pose pose = robot_->getPose();
  create::Vel vel = robot_->getVel();

  // Populate position info
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(0.0, 0.0, pose.yaw);
  geometry_msgs::msg::Quaternion quat = tf2::toMsg(tf_quat);
  odom_msg_.header.stamp = now();
  odom_msg_.pose.pose.position.x = pose.x;
  odom_msg_.pose.pose.position.y = pose.y;
  odom_msg_.pose.pose.orientation = quat;

  // Populate velocity info
  odom_msg_.twist.twist.linear.x = vel.x;
  odom_msg_.twist.twist.linear.y = vel.y;
  odom_msg_.twist.twist.angular.z = vel.yaw;

  // Update covariances
  odom_msg_.pose.covariance[0] = static_cast<double>(pose.covariance[0]);
  odom_msg_.pose.covariance[1] = pose.covariance[1];
  odom_msg_.pose.covariance[5] = pose.covariance[2];
  odom_msg_.pose.covariance[6] = pose.covariance[3];
  odom_msg_.pose.covariance[7] = pose.covariance[4];
  odom_msg_.pose.covariance[11] = pose.covariance[5];
  odom_msg_.pose.covariance[30] = pose.covariance[6];
  odom_msg_.pose.covariance[31] = pose.covariance[7];
  odom_msg_.pose.covariance[35] = pose.covariance[8];
  odom_msg_.twist.covariance[0] = vel.covariance[0];
  odom_msg_.twist.covariance[1] = vel.covariance[1];
  odom_msg_.twist.covariance[5] = vel.covariance[2];
  odom_msg_.twist.covariance[6] = vel.covariance[3];
  odom_msg_.twist.covariance[7] = vel.covariance[4];
  odom_msg_.twist.covariance[11] = vel.covariance[5];
  odom_msg_.twist.covariance[30] = vel.covariance[6];
  odom_msg_.twist.covariance[31] = vel.covariance[7];
  odom_msg_.twist.covariance[35] = vel.covariance[8];

  if (publish_tf_)
  {
    tf_odom_.header.stamp = now();
    tf_odom_.transform.translation.x = pose.x;
    tf_odom_.transform.translation.y = pose.y;
    tf_odom_.transform.rotation = quat;
    tf_broadcaster_.sendTransform(tf_odom_);
  }

  odom_pub_->publish(odom_msg_);
}

void CreateDriver::publishJointState()
{
  // Publish joint states
  float wheelRadius = model_.getWheelDiameter() / 2.0;

  joint_state_msg_.header.stamp = now();
  joint_state_msg_.position[0] = robot_->getLeftWheelDistance() / wheelRadius;
  joint_state_msg_.position[1] = robot_->getRightWheelDistance() / wheelRadius;
  joint_state_msg_.velocity[0] = robot_->getRequestedLeftWheelVel() / wheelRadius;
  joint_state_msg_.velocity[1] = robot_->getRequestedRightWheelVel() / wheelRadius;
  wheel_joint_pub_->publish(joint_state_msg_);
}

void CreateDriver::spinOnce()
{
  const auto spin_start = now();

  update();
  diagnostics_.force_update();

  // Check if the spin took longer than the target loop period.
  const auto spin_end = now();
  const auto elapsed = spin_end - spin_start;
  const double target_period = 1. / loop_hz_;
  is_running_slowly_ = elapsed.seconds() > target_period;

  if (is_running_slowly_)
  {
    RCLCPP_WARN(get_logger(), "[CREATE] Loop running slowly.");
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto create_driver = std::make_shared<CreateDriver>();

  try
  {
    rclcpp::spin(create_driver);
  }
  catch (std::runtime_error& ex)
  {
    RCLCPP_FATAL_STREAM(create_driver->get_logger(), "[CREATE] Runtime error: " << ex.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
