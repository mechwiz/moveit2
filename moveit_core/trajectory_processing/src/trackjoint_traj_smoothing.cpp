/* Author: Jack Center, Wyatt Rees, Andy Zelenak */

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <limits>
#include <moveit/trajectory_processing/trackjoint_traj_smoothing.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace trajectory_processing
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_trajectory_processing.trackjoint_traj_smoothing");
constexpr double DEFAULT_MAX_VELOCITY = 5;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 20;          // rad/s^3

bool TrackJointSmoothing::applySmoothing(robot_trajectory::RobotTrajectory& trajectory,
                                         const double max_velocity_scaling_factor,
                                         const double max_acceleration_scaling_factor) const
{
  return true;
}
}  // namespace trajectory_processing
