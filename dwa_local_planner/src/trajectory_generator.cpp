#include <math.h>
#include <ros/console.h>

#include <dwa_local_planner/trajectory_generator.h>

namespace dwa_local_planner
{

bool TrajectoryGenerator::hasMoreTrajectories()
{
  if (trajectory_counter_ > 0)
    return true;

  return false;
}

bool TrajectoryGenerator::nextTrajectory(base_local_planner::Trajectory &traj)
{
  if (trajectory_counter_ == 0)
  {
    ROS_WARN("trajectory counter is zero");
    return false;
  }

  trajectory_counter_--;
  traj = trajectories_[trajectory_counter_];
  return true;
}

void TrajectoryGenerator::initialise(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel,
                                     const Eigen::Vector3f& goal, base_local_planner::LocalPlannerLimits* limits,
                                     const Eigen::Vector3f& vsamples, bool discretize_by_time)
{
  ROS_INFO("New trajectory generator is used");

  trajectory_steps_ = 5;
  time_step_ = 1.0 / 4.0;

  dv_ = 1;
  dw_ = 3;

  v_max_ = limits->max_trans_vel;
  w_max_ = limits->max_rot_vel;

  ROS_INFO("pos: %f %f %f", pos[0], pos[1], pos[2]);
  ROS_INFO("vel: %f %f %f", vel[0], vel[1], vel[2]);

  PointState start_point;
  start_point.x = pos[0];
  start_point.y = pos[1];
  start_point.o = pos[2];
  start_point.v = sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
  start_point.w = vel[2];

  ros::Time::init();
  ros::Time d = ros::Time::now();
  
  trajectories_ = setup_branches(start_point, 0, 0, 0, 0);
  resverse_trajectories();
  trajectory_counter_ = trajectories_.size();

  ROS_INFO("New trajectory generator is used, trajectory count: %i", trajectory_counter_);
  std::cout << "time: " << ros::Time::now() - d << std::endl;
}

std::vector<base_local_planner::Trajectory> TrajectoryGenerator::setup_branches(PointState& point_state_old, double dv,
                                                                                double dw, double dt, int step_index)
{
  PointState point = calc_new_position(point_state_old, dv, dw, dt);

  if (step_index == trajectory_steps_)
  {
//    ROS_INFO("Endpoint x, y, o: %f, %f, %f", point.x, point.y, point.o);
    base_local_planner::Trajectory trajectory;
    trajectory.addPoint(point.x, point.y, point.o);
    trajectory.time_delta_ = 1.0 / 30.0;

    std::vector < base_local_planner::Trajectory > trajectories;
    trajectories.push_back(trajectory);

    return trajectories;
  }

  step_index++;

  std::vector < base_local_planner::Trajectory > trajectories;
  std::vector < base_local_planner::Trajectory > temp_trajectories;

  dt = time_step_;
  
//   if(step_index == 1)
//       dt = 1.0 / 10.0;
  
  temp_trajectories = setup_branches(point, -dv_, dw_, dt, step_index); //right turn
  trajectories.insert(trajectories.end(), temp_trajectories.begin(), temp_trajectories.end());

  temp_trajectories = setup_branches(point, -dv_, -dw_, dt, step_index); //left turn
  trajectories.insert(trajectories.end(), temp_trajectories.begin(), temp_trajectories.end());

  temp_trajectories = setup_branches(point, dv_, 0, dt, step_index); //straight fast
  trajectories.insert(trajectories.end(), temp_trajectories.begin(), temp_trajectories.end());

  temp_trajectories = setup_branches(point, -dv_, 0, dt, step_index); //straight slow
  trajectories.insert(trajectories.end(), temp_trajectories.begin(), temp_trajectories.end());

  for (int i = 0; i < trajectories.size(); ++i)
  {
    if(step_index == 3)
    {
      trajectories[i].xv_ = point.v;
      trajectories[i].yv_ = 0;
      trajectories[i].thetav_ = point.w;
    }

      
    if(step_index == 2)
    {
        if(point.v == 0)
        {
            trajectories[i].xv_ = 0;
        }
        else
        {
            trajectories[i].xv_ = point.v + trajectories[i].xv_;
        }        
      
      trajectories[i].yv_ = 0;
      trajectories[i].thetav_ = point.w  + trajectories[i].thetav_;
    }

    trajectories[i].addPoint(point.x, point.y, point.o);
  }

  return trajectories;
}

TrajectoryGenerator::PointState TrajectoryGenerator::calc_new_position(PointState& point_state_old, double dv,
                                                                       double dw, double dt)
{
  PointState new_point;

  double v = point_state_old.v + dv * dt;
  double w = point_state_old.w + dw * dt;

  if (v > v_max_)
  {
    dv = (v_max_ - point_state_old.v) / dt;
    v = v_max_;
  }
  else if (v < 0)
  {
    dv = -point_state_old.v / dt;
    v = 0;
  }

  if (w > w_max_)
  {
    dw = (w_max_ - point_state_old.w) / dt;
    w = w_max_;
  }
  else if (w < -w_max_)
  {
    dw = (-w_max_ - point_state_old.w) / dt;
    w = -w_max_;
  }

  new_point.v = v;
  new_point.w = w;

  new_point.o = fmod(point_state_old.o + point_state_old.w * dt + 0.5 * dw * dt * dt, 2 * 3.14159265359);
  double linear_distance = (point_state_old.v * dt + 0.5 * dv * dt * dt);
  new_point.x = point_state_old.x + cos(new_point.o) * linear_distance;
  new_point.y = point_state_old.y + sin(new_point.o) * linear_distance;

  return new_point;
}

void TrajectoryGenerator::resverse_trajectories()
{
  double x;
  double y;
  double o;

  std::vector < base_local_planner::Trajectory > reversed_trajectories;

  while (!trajectories_.empty())
  {
    base_local_planner::Trajectory trajectory;
    for (int p = trajectories_.back().getPointsSize() - 1; p >= 0; --p)
    {
      trajectories_.back().getPoint(p, x, y, o);
      trajectory.addPoint(x, y, o);
//      ROS_INFO("x, y, o: %f, %f, %f", x, y, o);
    }
    trajectory.xv_ = trajectories_.back().xv_;
    trajectory.yv_ = trajectories_.back().yv_;
    trajectory.thetav_ = trajectories_.back().thetav_;

    trajectories_.pop_back();
    reversed_trajectories.push_back(trajectory);
//    ROS_INFO("Trajectory xv, yv, w: %f, %f, %f", trajectory.xv_, trajectory.yv_, trajectory.thetav_);
  }

  trajectories_ = reversed_trajectories;
}

}
