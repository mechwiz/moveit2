/* Author: Jack Center, Wyatt Rees, Andy Zelenak */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/trackjoint_traj_smoothing.h>
#include <class_loader/class_loader.hpp>

namespace default_planner_request_adapters
{
using namespace trajectory_processing;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.add_traj_smoothing");

/** @brief This adapter uses the time-optimal trajectory generation method */
class AddTrackJointTrajectorySmoothing : public planning_request_adapter::PlanningRequestAdapter
{
public:
  AddTrackJointTrajectorySmoothing() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
  }

  std::string getDescription() const override
  {
    return "Add Trajectory Smoothing";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    bool result = planner(planning_scene, req, res);
    if (result && res.trajectory_)
    {
      RCLCPP_DEBUG(LOGGER, " Running '%s'", getDescription().c_str());
      TrackJointSmoothing smoother;
      if (!smoother.applySmoothing(*res.trajectory_, req.max_velocity_scaling_factor,
                                   req.max_acceleration_scaling_factor))
      {
        RCLCPP_WARN(LOGGER, " Trajectory smoothing for the solution path failed.");
        result = false;
      }
    }

    return result;
  }
};

}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::AddTrackJointTrajectorySmoothing,
                            planning_request_adapter::PlanningRequestAdapter)
