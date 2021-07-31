/* Author: Jack Center, Wyatt Rees, Andy Zelenak */

#pragma once

#include <Eigen/Core>
#include <list>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace trajectory_processing
{
class TrackJointSmoothing
{
public:
  bool applySmoothing(robot_trajectory::RobotTrajectory& trajectory, const double max_velocity_scaling_factor = 1.0,
                      const double max_acceleration_scaling_factor = 1.0) const;
};
}  // namespace trajectory_processing
