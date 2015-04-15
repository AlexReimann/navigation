#ifndef TREE_LOCAL_PLANNER_TREE_PLANNER_ROS_H_
#define TREE_LOCAL_PLANNER_TREE_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <dynamic_reconfigure/server.h>
#include <tree_local_planner/TreePlannerConfig.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>


#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <tree_local_planner/backward_RRT.h>

#include <tree_local_planner/tree_planner.h>
#include <ompl_visualization/ompl_visualization.h>

namespace tree_local_planner
{

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
  const ob::SE2StateSpace::StateType *se2state = start->as<ob::SE2StateSpace::StateType>();
  const double* pos = se2state->as < ob::RealVectorStateSpace::StateType > (0)->values;
  const double rot = se2state->as < ob::SO2StateSpace::StateType > (1)->value;
  const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

  result->as<ob::SE2StateSpace::StateType>()->setXY(pos[0] + ctrl[0] * duration * cos(rot),
                                                    pos[1] + ctrl[0] * duration * sin(rot));
  result->as<ob::SE2StateSpace::StateType>()->setYaw(rot + ctrl[1] * duration);
}

/**
 * @class DWAPlannerROS
 * @brief ROS Wrapper for the DWAPlanner that adheres to the
 * BaseLocalPlanner interface and can be used as a plugin for move_base.
 */
class TreePlannerROS : public nav_core::BaseLocalPlanner
{
public:
  /**
   * @brief  Constructor for DWAPlannerROS wrapper
   */
  TreePlannerROS();

  /**
   * @brief  Constructs the ros wrapper
   * @param name The name to give this instance of the trajectory planner
   * @param tf A pointer to a transform listener
   * @param costmap The cost map to use for assigning costs to trajectories
   */
  void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief  Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid trajectory was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  void start();

  /**
   * @brief  Set the plan that the controller is following
   * @param orig_global_plan The plan to pass to the controller
   * @return True if the plan was updated successfully, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool isGoalReached();

  bool isStateValid(const oc::SpaceInformation *si, const ob::State *state);

private:
  /**
   * @brief Callback to update the local planner's parameters based on dynamic reconfigure
   */
  void reconfigureCB(TreePlannerConfig &config, uint32_t level);

  ompl::control::SpaceInformationPtr setup_space_information();
  bool update_local_goal_from_global_plan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  inline int local_to_costmap(double coordinate)
  {
    coordinate += 2;
    coordinate /= costmap_ros_->getCostmap()->getResolution();
    return coordinate;
  }

  double yawFromQuaternion(tf::Quaternion quaternion)
  {
    tf::Matrix3x3 m(quaternion);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_ERROR("roll, pitch, yaw: %f %f %f", roll, pitch, yaw);
    return yaw;
  }

  costmap_2d::Costmap2DROS* costmap_ros_;
  ompl::base::PlannerPtr planner_;
  tf::TransformListener tf_listener_;

  bool goal_reached_;
  geometry_msgs::PoseStamped local_goal_;
  ompl_visualization::OmplVisualization ompl_visualizer_;
  bool found_path_;
};
}
;
#endif
