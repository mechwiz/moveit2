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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace moveit_servo
{
namespace
{
static const std::string PARAMETER_NS = "moveit_servo";
}

class LatencyMonitor : public rclcpp::Node
{
public:
  LatencyMonitor() : Node("latency_monitor")
  {
    this->declare_parameter<double>(PARAMETER_NS + ".publish_period");
    this->get_parameter<double>(PARAMETER_NS + ".publish_period", servo_period_);
    this->declare_parameter<std::string>(PARAMETER_NS + ".ee_frame_name");
    this->get_parameter<std::string>(PARAMETER_NS + ".ee_frame_name", ee_frame_name_);
    this->declare_parameter<std::string>(PARAMETER_NS + ".command_out_topic");
    this->get_parameter<std::string>(PARAMETER_NS + ".command_out_topic", joint_position_command_topic_);

    // TODO: re-enable this validation when done testing in sim

    // Verify that the command type out of Servo is compatible with this node
    // Currently compatible only when Servo publishes std_msgs/Float64MultiArray position commands.
    //    this->declare_parameter<std::string>(PARAMETER_NS + ".command_out_type");
    //    this->get_parameter<std::string>(PARAMETER_NS + ".command_out_type", command_out_type_);
    //    if (command_out_type_ != "std_msgs/Float64MultiArray")
    //    {
    //      RCLCPP_ERROR_STREAM(this->get_logger(),
    //                          "Latency monitor is only compatible with Float64MultiArray-type position commands.");
    //      std::exit(EXIT_FAILURE);
    //    }
    //    this->declare_parameter<bool>(PARAMETER_NS + ".publish_joint_positions");
    //    this->get_parameter<bool>(PARAMETER_NS + ".publish_joint_positions", publish_joint_positions_);
    //    if (!publish_joint_positions_)
    //    {
    //      RCLCPP_ERROR_STREAM(this->get_logger(),
    //                          "Latency monitor is only compatible with Float64MultiArray-type position commands.");
    //      std::exit(EXIT_FAILURE);
    //    }

    timer_ = this->create_wall_timer(std::chrono::duration<double>(servo_period_),
                                     std::bind(&LatencyMonitor::timer_callback, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribe to latest command from Servo
    servo_command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        joint_position_command_topic_, rclcpp::SystemDefaultsQoS(),
        [this](std_msgs::msg::Float64MultiArray::UniquePtr msg) {
          RCLCPP_INFO_STREAM(this->get_logger(), "I heard a joint command");
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

    // Get expected robot position from Servo commands

    // Calculate a running average latency

    // Estimate latency like this:
    // delta_x_actual = (servo_period - latency) * velocity_command
    // delta_x_cmd = servo_period * velocity_command
    // delta_x_cmd - delta_x_actual = latency * velocity_command
    // -->  latency = (delta_x_cmd - delta_x_actual) / velocity_command
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr servo_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr servo_status_sub_;

  std_msgs::msg::Int8 latest_servo_status_;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{ nullptr };
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Parameters
  double servo_period_;
  std::string ee_frame_name_;
  std::string joint_position_command_topic_;
  std::string command_out_type_;
  bool publish_joint_positions_;
};
}  // namespace moveit_servo

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<moveit_servo::LatencyMonitor>());
  rclcpp::shutdown();
  return 0;
}
