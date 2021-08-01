/* Author: Jack Center, Wyatt Rees, Andy Zelenak */

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>
#include <limits>
#include <moveit/trajectory_processing/trackjoint_traj_smoothing.h>
#include <rclcpp/rclcpp.hpp>
#include <trackjoint/trajectory_generator.h>
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
  const moveit::core::JointModelGroup* group = trajectory.getGroup();
  if (!group)
  {
    RCLCPP_ERROR(LOGGER, "It looks like the planner did not set the group the plan was computed for");
    return false;
  }

  const size_t num_waypoints = trajectory.getWayPointCount();
  if (num_waypoints < 2)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Trajectory does not have enough points to smooth with Ruckig");
    return false;
  }

  // This lib does not actually work properly when angles wrap around, so we need to unwind the path first
  trajectory.unwind();

  const auto kNumDof = group->getActiveJointModels().size();
  constexpr double kMaxDuration = 5;
  const auto kTimestep = trajectory.getAverageSegmentDuration();
  // TODO(andyz): this is pretty large. It helps (temporarily) skip noisy derivative calcs at the first waypoint
  constexpr double kPositionTolerance = 0.02;  // radians
  constexpr bool kUseHighSpeedMode = false;

  std::vector<trackjoint::Limits> limits(kNumDof);
  trackjoint::Limits single_joint_limits;
  single_joint_limits.velocity_limit = 3.15;  // To match value in joint_limits.yaml
  single_joint_limits.acceleration_limit = 5;
  single_joint_limits.jerk_limit = 20000;
  limits[0] = single_joint_limits;
  limits[1] = single_joint_limits;
  limits[2] = single_joint_limits;
  limits[3] = single_joint_limits;
  limits[4] = single_joint_limits;
  limits[5] = single_joint_limits;

  // Vector of start states - for each waypoint, for each joint
  std::vector<std::vector<trackjoint::KinematicState>> trackjt_current_joint_states(kNumDof);
  // Vector of goal states - for each waypoint, for each joint
  std::vector<std::vector<trackjoint::KinematicState>> trackjt_goal_joint_states(kNumDof);
  std::vector<double> trackjt_desired_durations(kNumDof);
  trackjoint::KinematicState joint_state;

  // For each incoming waypoint
  for (std::size_t point = 0; point < num_waypoints - 1; ++point)
  {
    ;
  }

  return true;
}
}  // namespace trajectory_processing
