#include <tree_local_planner/tree_planner_ros.h>
#include <Eigen/Core>
#include <cmath>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


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
  planner_ = ompl::base::PlannerPtr(new ompl::control::RRT(setup_space_information()));
  planner_->setup();
}

ompl::control::SpaceInformationPtr TreePlannerROS::setup_space_information()
{
  ompl::base::StateSpacePtr work_space(new ompl::base::SE2StateSpace());

  ompl::base::RealVectorBounds work_space_bounds(2);
  //set bounds for both dimensions (squared area)
  work_space_bounds.setLow(-2);
  work_space_bounds.setHigh(2);

  work_space->as<ompl::base::SE2StateSpace>()->setBounds(work_space_bounds);

  ompl::control::ControlSpacePtr control_space(new ompl::control::RealVectorControlSpace(work_space, 2));

  ompl::base::RealVectorBounds control_space_bounds(2);
  //set linear velocity bounds
//  control_space_bounds.setLow(0.0);
//  control_space_bounds.setHigh(1.0);
  control_space_bounds.setLow(0, 0.0); // can't go backwards
  control_space_bounds.setHigh(0, 1.0);

  //set angular velocity bounds
  control_space_bounds.setLow(1, -3.5); // can't go backwards
  control_space_bounds.setHigh(1, 3.5);

  control_space->as<ompl::control::RealVectorControlSpace>()->setBounds(control_space_bounds);

  ompl::control::SpaceInformationPtr space_information(new ompl::control::SpaceInformation(work_space, control_space));

  space_information->setStateValidityChecker(
      boost::bind(&TreePlannerROS::isStateValid, this, space_information.get(), _1));
  space_information->setStatePropagator(boost::bind(&propagate, _1, _2, _3, _4));

  return space_information;
}

bool TreePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  goal_reached_ = false;
  return update_local_goal_from_global_plan(orig_global_plan);
}

bool TreePlannerROS::update_local_goal_from_global_plan(const std::vector<geometry_msgs::PoseStamped>& global_plan)
{
  if(global_plan.size() == 0)
    return false;

  std::vector<geometry_msgs::PoseStamped>::const_iterator global_plan_it = global_plan.begin();

  tf::Stamped<tf::Pose> costmap_robot_pose;
  costmap_ros_->getRobotPose(costmap_robot_pose);

  tf::Stamped<tf::Pose> global_robot_pose;
  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
  tf_listener_.transformPose(plan_pose.header.frame_id, costmap_robot_pose, global_robot_pose);

  double x = global_robot_pose.getOrigin().getX();
  double y = global_robot_pose.getOrigin().getY();

  ROS_INFO("x: %f; y: %f", x, y);

  double dx = 0;
  double dy = 0;

  ROS_INFO("global_plan size: %i", (int)global_plan.size());

  while (global_plan_it != global_plan.end())
  {
    dx = fabs(global_plan_it->pose.position.x - x);
    dy = fabs(global_plan_it->pose.position.y - y);

    if (dx >= 2 || dy >= 2)
    {

//      ROS_INFO("local goal search ended at local map border");
      return true;;
    }

//    ROS_INFO("dx: %f; dy: %f; pos: %f, %f", dx, dy, global_plan_it->pose.position.x, global_plan_it->pose.position.y);
    global_local_goal_.x = global_plan_it->pose.position.x;
    global_local_goal_.y = global_plan_it->pose.position.y;
    temp_goal_ = *global_plan_it;

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

  if (x < -2 || x > 2 || y < -2 || y > 2)
    return false;

  ///TODO test if footprint hits obstacle
  if (costmap_ros_->getCostmap()->getCost(local_to_costmap(x), local_to_costmap(y)) == costmap_2d::LETHAL_OBSTACLE)
  {
    ROS_ERROR("Path hit obstacle!");
    return false;
  }

  // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
  return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}

bool TreePlannerROS::isGoalReached()
{
  return goal_reached_;
}

geometry_msgs::Point TreePlannerROS::get_current_local_goal()
{
  tf::Stamped<tf::Pose> costmap_robot_pose;
  costmap_ros_->getRobotPose(costmap_robot_pose);

  tf::Stamped<tf::Pose> global_robot_pose;
  ///TODO dynamic frame id
  tf_listener_.transformPose("map", costmap_robot_pose, global_robot_pose);

  geometry_msgs::Point local_goal;
  local_goal.x = global_robot_pose.getOrigin().getX() - global_local_goal_.x;
  local_goal.y = global_robot_pose.getOrigin().getY() - global_local_goal_.y;

  temp_goal_.header.stamp = ros::Time::now();

  try {
    tf_listener_.waitForTransform("base_link", "map", temp_goal_.header.stamp, ros::Duration(1.0) );
  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
  }

  geometry_msgs::PoseStamped goal_local;
  tf_listener_.transformPose("base_link", temp_goal_, goal_local);

  local_goal.x = goal_local.pose.position.x;
  local_goal.y = goal_local.pose.position.y;

  return local_goal;
}

bool TreePlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  tf::Stamped<tf::Pose> current_pose;
  costmap_ros_->getRobotPose(current_pose);

  planner_->clear();
  ompl::base::StateSpacePtr work_space = planner_->getSpaceInformation()->getStateSpace();

  double angle_robot = atan2(current_pose.getOrigin().getY(), current_pose.getOrigin().getX());

  ob::ScopedState < ob::SE2StateSpace > start(work_space);
  //always in the center
  start->setX(0.0);
  start->setY(0.0);
  start->setYaw(0);

  geometry_msgs::Point local_goal = get_current_local_goal();

  if(fabs(local_goal.x) < 0.1  && fabs(local_goal.y) < 0.1)
  {
    ROS_ERROR("Goal reached");
    goal_reached_ = true;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    return true;
  }

  double angle_goal = atan2(local_goal.y, local_goal.x);

  ob::ScopedState < ob::SE2StateSpace > goal(work_space);
  goal->setX(local_goal.x);
  goal->setY(local_goal.y);
  start->setYaw(0);

  ompl_visualizer_.add_big_marker("base_link", 0, local_goal.x, local_goal.y, 0, 0, 1, 0);

//  ROS_ERROR("RRT goal x, y, w: %f %f %f", local_goal.x, local_goal.y, angle_goal);

  ob::ProblemDefinitionPtr planner_problem(new ob::ProblemDefinition(planner_->getSpaceInformation()));
  double goal_found_epsilon = 0.01;
  planner_problem->setStartAndGoalStates(start, goal, goal_found_epsilon);
  planner_->setProblemDefinition(planner_problem);

  double max_time = 0.1;
  ob::PlannerStatus solved = planner_->solve(max_time);

  if(solved)
  {
    ROS_WARN("local path found!");

    oc::PathControl local_path = static_cast<oc::PathControl&>(*(planner_problem->getSolutionPath()));
    local_path.interpolate();

    oc::RealVectorControlSpace::ControlType* control = local_path.getControls()[0]->as<oc::RealVectorControlSpace::ControlType>();

    cmd_vel.linear.x = (*control)[0];
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = (*control)[1];
//    ROS_ERROR("Velocity commands x, angular: %f, %f", cmd_vel.linear.x, cmd_vel.angular.z);

    ompl_visualizer_.visualize_path_states(local_path.getStates(), "base_link");
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
