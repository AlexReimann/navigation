#ifndef DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_
#define DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_

#include <vector>

#include <base_local_planner/trajectory_sample_generator.h>
#include <base_local_planner/local_planner_limits.h>
#include <Eigen/Core>

namespace dwa_local_planner
{

class TrajectoryGenerator : public base_local_planner::TrajectorySampleGenerator
{
  struct PointState
  {
    double x;
    double y;
    double o;
    double v;
    double w;
  };


public:

  /**
   * Whether this generator can create more trajectories
   */
  bool hasMoreTrajectories();

  /**
   * Whether this generator can create more trajectories
   */
  bool nextTrajectory(base_local_planner::Trajectory &traj);

  ///temp
  void setParameters(double sim_time, double sim_granularity, double angular_sim_granularity, bool use_dwa = false,
                     double sim_period = 0.0)
  {
  }

  void initialise(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, const Eigen::Vector3f& goal,
                  base_local_planner::LocalPlannerLimits* limits, const Eigen::Vector3f& vsamples,
                  bool discretize_by_time = false);

  bool generateTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f sample_target_vel,
                          base_local_planner::Trajectory& traj)
  {
    ///generate trajectory (which one? -> selected by sample_target_velocity) in here with given pos, vel
    ///only called on target reached or and rotation into end position or something
    return false;
  }

protected:
  int trajectory_steps_;
  double time_step_;

  double dv_;
  double dw_;

  double v_max_;
  double w_max_;

  int trajectory_counter_;
  std::vector<base_local_planner::Trajectory> trajectories_;

  std::vector<base_local_planner::Trajectory> setup_branches(PointState& point_state_old, double dv, double dw, double dt, int step_index);
  PointState calc_new_position(PointState& point_state_old, double dv, double dw, double dt);

  void resverse_trajectories();
};

}

#endif /* DWA_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H_ */
