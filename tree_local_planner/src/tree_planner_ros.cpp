#include <tree_local_planner/tree_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sstream>

namespace tree_local_planner
{

void TreePlannerROS::reconfigureCB(TreePlannerConfig &config, uint32_t level)
{
//  if (setup_ && config.restore_defaults)
//  {
//    config = default_config_;
//    config.restore_defaults = false;
//  }
}

TreePlannerROS::TreePlannerROS()
{

}

void TreePlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  costmap_ros_ = costmap_ros;

//  planner_ = ompl::base::PlannerPtr(new ompl::geometric::LBTRRT(setup_space_information()));
//  planner_ = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(setup_space_information()));
//  planner_ = ompl::base::PlannerPtr(new ompl::control::RRT(setup_space_information()));
  planner_ = ompl::base::PlannerPtr(new backward_rrt::BackwardRRT(setup_space_information()));
  planner_->setup();
}

ompl::control::SpaceInformationPtr TreePlannerROS::setup_space_information()
{
  ompl::base::StateSpacePtr work_space(new ompl::base::SE2StateSpace());

  ompl::base::RealVectorBounds work_space_bounds(2);
  //set bounds for both dimensions (squared area)
  work_space_bounds.setLow(-30);
  work_space_bounds.setHigh(30);

  work_space->as<ompl::base::SE2StateSpace>()->setBounds(work_space_bounds);

  ompl::control::ControlSpacePtr control_space(new ompl::control::RealVectorControlSpace(work_space, 2));

  ompl::base::RealVectorBounds control_space_bounds(2);
  //set linear velocity bounds
//  control_space_bounds.setLow(0.0);
//  control_space_bounds.setHigh(1.0);
  control_space_bounds.setLow(0, -1.0); // can't go backwards
  control_space_bounds.setHigh(0, 0.0);

  //set angular velocity bounds
  control_space_bounds.setLow(1, -3.5); // can't go backwards
  control_space_bounds.setHigh(1, 3.5);

  control_space->as<ompl::control::RealVectorControlSpace>()->setBounds(control_space_bounds);

  ompl::control::SpaceInformationPtr space_information(new ompl::control::SpaceInformation(work_space, control_space));

  space_information->setStateValidityChecker(
      boost::bind(&TreePlannerROS::isStateValid, this, space_information.get(), _1));
  space_information->setStatePropagator(boost::bind(&propagate, _1, _2, _3, _4));
  space_information->setPropagationStepSize(0.07);
  space_information->setup();

  return space_information;
}

bool TreePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  goal_reached_ = false;
  found_path_ = false;

  static_cast<backward_rrt::BackwardRRT&>(*planner_).reset_next_neigbhours();

  return update_local_goal_from_global_plan(orig_global_plan);
}

bool TreePlannerROS::update_local_goal_from_global_plan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
  if (global_plan.size() == 0)
    return false;

  std::vector<geometry_msgs::PoseStamped>::const_iterator global_plan_it = global_plan.begin();

  tf::Stamped<tf::Pose> costmap_robot_pose;
  costmap_ros_->getRobotPose(costmap_robot_pose);

  tf::Stamped<tf::Pose> global_robot_pose;
  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
  tf_listener_.transformPose(plan_pose.header.frame_id, costmap_robot_pose, global_robot_pose);

  double x = global_robot_pose.getOrigin().getX();
  double y = global_robot_pose.getOrigin().getY();

  ROS_INFO("global_plan size: %i", (int)global_plan.size());

  double dx = 0;
  double dy = 0;

  while (global_plan_it != global_plan.end())
  {
    dx = fabs(global_plan_it->pose.position.x - x);
    dy = fabs(global_plan_it->pose.position.y - y);

    if (dx >= 2 || dy >= 2)
      return true;

    local_goal_ = *global_plan_it;

    global_plan_it++;
  }

//  ROS_INFO("Local goal inside local map found");
  return true;
}

bool TreePlannerROS::isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
  //    ob::ScopedState<ob::SE2StateSpace>
  // cast the abstract state type to the type we expect
  const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const ob::RealVectorStateSpace::StateType* pos = se2state->as < ob::RealVectorStateSpace::StateType > (0);

  // extract the second component of the state and cast it to what we expect
  const ob::SO2StateSpace::StateType *rot = se2state->as < ob::SO2StateSpace::StateType > (1);

  double x = (*pos)[0];
  double y = (*pos)[1];

//  ROS_WARN("isStateValid");
//  ROS_INFO("original x, y: %f %f", x, y);
//  ROS_INFO("transformed x, y: %i %i", local_to_costmap(x), local_to_costmap(y));

  if (x < -30 || x > 30 || y < -30 || y > 30)
    return false;

  ///TODO test if footprint hits obstacle
//  if (costmap_ros_->getCostmap()->getCost(local_to_costmap(x), local_to_costmap(y)) == costmap_2d::LETHAL_OBSTACLE)
//  {
//    ROS_ERROR("Path hit obstacle!");
//    return false;
//  }

  // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
  return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}

bool TreePlannerROS::isGoalReached()
{
  return goal_reached_;
}

bool TreePlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  tf::Stamped<tf::Pose> local_robot_pose;
  costmap_ros_->getRobotPose(local_robot_pose);

  tf::Stamped<tf::Pose> global_robot_pose;
  tf_listener_.transformPose(local_goal_.header.frame_id, local_robot_pose, global_robot_pose);

  planner_->clear();

  if(!found_path_)
  {
    static_cast<backward_rrt::BackwardRRT&>(*planner_).setGoalBias(0.05);
    static_cast<backward_rrt::BackwardRRT&>(*planner_).reset_next_neigbhours();
  }
  else
  {
    static_cast<backward_rrt::BackwardRRT&>(*planner_).setGoalBias(0.3);
  }

  ompl::base::StateSpacePtr work_space = planner_->getSpaceInformation()->getStateSpace();

  ob::ScopedState < ob::SE2StateSpace > start(work_space);
  //always in the center
  start->setX(global_robot_pose.getOrigin().getX());
  start->setY(global_robot_pose.getOrigin().getY());
  start->setYaw(yawFromQuaternion(local_robot_pose.getRotation()));

  tf::Quaternion goal_angle_quaternion;
  tf::quaternionMsgToTF(local_goal_.pose.orientation, goal_angle_quaternion);

  ob::ScopedState < ob::SE2StateSpace > goal(work_space);
  goal->setX(local_goal_.pose.position.x);
  goal->setY(local_goal_.pose.position.y);
  goal->setYaw(yawFromQuaternion(goal_angle_quaternion));
  ///TODO correct angle goal

  if (fabs(goal->getX() - start->getX()) < 0.1 && fabs(goal->getY() - start->getY()) < 0.1)
  {
    ROS_ERROR("Goal reached");
    goal_reached_ = true;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }

  static_cast<backward_rrt::BackwardRRT&>(*planner_).update_start_if_needed(&(*goal));

  ompl_visualizer_.add_big_marker("map", 0, goal->getX(), goal->getY(), 0, 0, 1, 0);

  ROS_ERROR("RRT start x, y, w: %f %f %f", start->getX(), start->getY(), start->getYaw());
  ROS_ERROR("RRT goal x, y, w: %f %f %f", goal->getX(), goal->getY(), goal->getYaw());

  ob::ProblemDefinitionPtr planner_problem(new ob::ProblemDefinition(planner_->getSpaceInformation()));
  double goal_found_epsilon = 0.1;
  planner_problem->setStartAndGoalStates(goal, start, goal_found_epsilon);
  planner_->setProblemDefinition(planner_problem);

  double max_time = 0.05;
  ob::PlannerStatus solved = planner_->solve(max_time);

  if (solved)
  {
    ROS_WARN("local path found!");

    oc::PathControl local_path = static_cast<oc::PathControl&>(*(planner_problem->getSolutionPath()));
    local_path.interpolate();

    oc::RealVectorControlSpace::ControlType* control =
        local_path.getControls()[local_path.getControls().size() - 1]->as<oc::RealVectorControlSpace::ControlType>();

    if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
      found_path_ = true;
      cmd_vel.linear.x = -(*control)[0];
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = -(*control)[1];

      static_cast<backward_rrt::BackwardRRT&>(*planner_).remove_goal(&(*start), local_path.getControls().size());
    }
    else
    {
      found_path_ = false;
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0;
      ROS_ERROR("No exact solution found, not moving");
    }
//    ROS_ERROR("Velocity commands x, angular: %f, %f", cmd_vel.linear.x, cmd_vel.angular.z);

    std::vector<ob::PlannerSolution> solutions = planner_problem->getSolutions();

    ompl_visualizer_.visualize_path_states(local_path.getStates(), "map", "ompl");

//    ROS_ERROR("solutions.size() %i", (int)solutions.size());
//
//    for(int i = 0; i < solutions.size(); ++i)
//    {
//      std::ostringstream Convert;
//
//      Convert << "path_";
//      Convert << i;
//      ompl_visualizer_.visualize_path_states(static_cast<oc::PathControl&>(*(solutions[i].path_)).getStates(), "map", Convert.str());
//    }
  }
  else
    ROS_ERROR("No path found");

//  const ob::PlannerDataPtr planner_data(new ob::PlannerData(planner_->getSpaceInformation()));
//  planner_->getPlannerData(*planner_data);
//  ompl_visualizer_.visualize(planner_data, "base_link");

  return solved;
}

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(tree_local_planner::TreePlannerROS, nav_core::BaseLocalPlanner)

}
;
