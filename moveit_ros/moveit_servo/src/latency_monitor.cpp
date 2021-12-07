/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : latency_monitor.cpp
 *      Project   : moveit_servo
 *      Created   : 12/6/2021
 *      Author    : Andy Zelenak
 *      Description: Read the actual robot position vs. Servo-expected robot position. Suggest what the latency
 *                   might be based on the difference. For now, this uses a simple running average.
 */

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace moveit_servo
{
namespace
{
static const std::string PARAMETER_NS = "moveit_servo";
static const std::string WORLD_FRAME = "world";
}  // namespace

class LatencyMonitor : public rclcpp::Node
{
public:
  LatencyMonitor() : Node("latency_monitor"), running_average_latency_(0), num_samples_(0)
  {
    this->declare_parameter<double>(PARAMETER_NS + ".publish_period");
    this->get_parameter<double>(PARAMETER_NS + ".publish_period", servo_period_);
    this->declare_parameter<std::string>(PARAMETER_NS + ".ee_frame_name");
    this->get_parameter<std::string>(PARAMETER_NS + ".ee_frame_name", ee_frame_name_);
    this->declare_parameter<std::string>(PARAMETER_NS + ".cartesian_command_in_topic");
    this->get_parameter<std::string>(PARAMETER_NS + ".cartesian_command_in_topic", twist_command_to_servo_topic_);

    timer_ = this->create_wall_timer(std::chrono::duration<double>(servo_period_),
                                     std::bind(&LatencyMonitor::timer_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to latest to Servo
    servo_command_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        twist_command_to_servo_topic_, rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::TwistStamped::UniquePtr msg) {
          RCLCPP_INFO_STREAM(this->get_logger(), "I heard a joint command");
          latest_twist_cmd_to_servo_ = *msg;
        });

    // Subscribe to Servo status. We only run the calculations if Servo is in perfect operating condition
    // (not near singularity, collision, or joint limit)
    servo_status_sub_ = this->create_subscription<std_msgs::msg::Int8>("servo_node/status", rclcpp::SystemDefaultsQoS(),
                                                                       [this](std_msgs::msg::Int8::UniquePtr msg) {
                                                                         latest_servo_status_ = *msg;
                                                                       });
  }

private:
  void timer_callback()
  {
    // Get actual robot position with tf
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_->lookupTransform(ee_frame_name_, WORLD_FRAME, tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", ee_frame_name_.c_str(), WORLD_FRAME.c_str(),
                  ex.what());
      return;
    }
    latest_ee_position_[0] = transform_stamped.transform.translation.x;
    latest_ee_position_[1] = transform_stamped.transform.translation.y;
    latest_ee_position_[2] = transform_stamped.transform.translation.z;

    // Get expected robot position from Servo commands

    // We only run the latency calculations if Servo is in perfect operating condition
    // (not near singularity, collision, or joint limit)
    if (latest_servo_status_.data == 0)
    {
      ++num_samples_;

      // Calculate a running average latency

      // Estimate latency like this:
      // delta_x_actual = (servo_period - latency) * velocity_command
      // delta_x_cmd = servo_period * velocity_command
      // delta_x_cmd - delta_x_actual = latency * velocity_command
      // -->  latency = (delta_x_cmd - delta_x_actual) / velocity_command
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr servo_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_sub_;

  // For now, we only track end-effector position
  Eigen::Vector3d latest_ee_position_;
  std_msgs::msg::Int8 latest_servo_status_;
  geometry_msgs::msg::TwistStamped latest_twist_cmd_to_servo_;
  double running_average_latency_;
  size_t num_samples_;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Parameters
  double servo_period_;
  std::string ee_frame_name_;
  std::string twist_command_to_servo_topic_;
};
}  // namespace moveit_servo

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<moveit_servo::LatencyMonitor>());
  rclcpp::shutdown();
  return 0;
}
