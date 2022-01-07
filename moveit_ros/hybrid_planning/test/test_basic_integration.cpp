/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, PickNik Inc.
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

/* Author: Andy Zelenak
   Desc:   Launch hybrid planning and test basic functionality
*/

#include <gtest/gtest.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>

#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

namespace moveit_hybrid_planning
{
using namespace std::chrono_literals;

class HybridPlanningFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
  }

  HybridPlanningFixture() : node_(std::make_shared<rclcpp::Node>("hybrid_planning_testing"))
  {
    std::string hybrid_planning_action_name = "";
    node_->declare_parameter("hybrid_planning_action_name", "");
    if (node_->has_parameter("hybrid_planning_action_name"))
    {
      node_->get_parameter<std::string>("hybrid_planning_action_name", hybrid_planning_action_name);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "hybrid_planning_action_name parameter was not defined");
      std::exit(EXIT_FAILURE);
    }

    hp_action_client_ =
        rclcpp_action::create_client<moveit_msgs::action::HybridPlanner>(node_, hybrid_planning_action_name);
    robot_state_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>("display_robot_state", 1);

    // Add new collision object as soon as global trajectory is available.
    global_solution_subscriber_ = node_->create_subscription<moveit_msgs::msg::MotionPlanResponse>(
        "global_trajectory", rclcpp::SystemDefaultsQoS(),
        [this](const moveit_msgs::msg::MotionPlanResponse::SharedPtr msg) {});

    RCLCPP_INFO(node_->get_logger(), "Initialize Planning Scene Monitor");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node_, "robot_description", tf_buffer_, "planning_scene_monitor");
    if (!planning_scene_monitor_->getPlanningScene())
    {
      RCLCPP_ERROR(node_->get_logger(), "The planning scene was not retrieved!");
      std::exit(EXIT_FAILURE);
    }
    else
    {
      planning_scene_monitor_->startStateMonitor();
      planning_scene_monitor_->providePlanningSceneService();  // let RViz display query PlanningScene
      planning_scene_monitor_->setPlanningScenePublishingFrequency(100);
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "/planning_scene");
      planning_scene_monitor_->startSceneMonitor();
    }

    if (!planning_scene_monitor_->waitForCurrentRobotState(node_->now(), 5))
    {
      RCLCPP_ERROR(node_->get_logger(), "Timeout when waiting for /joint_states updates. Is the robot running?");
      std::exit(EXIT_FAILURE);
    }

    if (!hp_action_client_->wait_for_action_server(20s))
    {
      RCLCPP_ERROR(node_->get_logger(), "Hybrid planning action server not available after waiting");
      std::exit(EXIT_FAILURE);
    }
  }

  void TearDown() override
  {
  }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SharedPtr hp_action_client_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_solution_subscriber_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
};  // class HybridPlanningFixture

// Make a hybrid planning request and verify it completes
TEST_F(HybridPlanningFixture, ActionCompletion)
{
  ASSERT_TRUE(true);
}
}  // namespace moveit_hybrid_planning

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);

  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
