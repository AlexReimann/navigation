#ifndef BACKWARDS_RRT_BACKWARDSRRT_H_
#define BACKWARDS_RRT_BACKWARDSRRT_H_

#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

namespace backward_rrt
{

class BackwardRRT : public ompl::control::RRT
{

public:
  BackwardRRT(const ompl::control::SpaceInformationPtr &si) :
      ompl::control::RRT(si)
  {

  }

  void clear()
  {
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    lastGoalMotion_ = NULL;
  }

  void reset_next_neigbhours()
  {
    freeMemory();
    if (nn_)
      nn_->clear();
  }

  void update_start_if_needed(ompl::base::State* new_start_state)
  {
    //make the motion object of the new start
    Motion* new_start = new Motion(siC_);
    si_->copyState(new_start->state, new_start_state);
    siC_->nullControl(new_start->control);

    if (nn_->size() == 0)
    {
      //first tree node, nothing special to do
      nn_->add(new_start);
      return;
    }
    //else add the new start as parent node of the old tree, if it is a new position

    ///TODO this might not always give the top node of the tree, no idea what happens then.
    Motion* old_start = nn_->nearest(new_start);

    if (old_start == NULL)
      ROS_ERROR("blubber");

    if (!same_state(static_cast<ompl::base::SE2StateSpace::StateType*>(new_start->state),
                    static_cast<ompl::base::SE2StateSpace::StateType*>(old_start->state)))
    {
      if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

      Motion *temp_motion = new Motion(siC_);

      /**
       * Calculate the trajectory from old_start to new_start to make sure we get a
       * don't break the tree:
       * The target point of the trajectory calculation (new_start) can vary a bit.
       * If this would be old_start, other trajectories in the tree might become invalid.
       *
       * Even if we don't reach the start point (= goal point) perfectly, we still can
       * adjust later.
       */

//      ROS_ERROR("happy happy, joy joy: %i", old_start);
//      ROS_ERROR("happy happy, joy joy: %i", temp_motion);
//      ROS_ERROR("happy happy, joy joy: %i", &(*controlSampler_));
      unsigned int cd = controlSampler_->sampleTo(temp_motion->control, old_start->control, old_start->state,
                                                  temp_motion->state);

      if (cd < siC_->getMinControlDuration())
      {
        return;
      }

      ompl::control::RealVectorControlSpace::ControlType* old_control =
          static_cast<ompl::control::RealVectorControlSpace::ControlType*>(old_start->control);
      ompl::control::RealVectorControlSpace::ControlType* temp_control =
          static_cast<ompl::control::RealVectorControlSpace::ControlType*>(temp_motion->control);

      // change the control of the old_start to get from new_start to old_start (opposite as calculated)
      (*old_control)[0] = (*temp_control)[0];
      (*old_control)[1] = -(*temp_control)[1]; //just the direction of angle changes
      old_start->steps = cd;
      old_start->parent = new_start;

      //adjust the position / state of the new start to the calculated
      si_->copyState(new_start->state, temp_motion->state);
      nn_->add(new_start);

      //rebuild to make sure the new_start is at the correct place in the tree
      (static_cast<ompl::NearestNeighborsGNAT<Motion*>&>(*nn_)).rebuildDataStructure();

      delete temp_motion;
    }
    //else we don't need to add a new start node because it is still the same
  }

  void remove_goal(ompl::base::State* goal_state, int max_size)
  {
    ROS_ERROR("remove");
    Motion* goal_motion = new Motion(siC_);
    si_->copyState(goal_motion->state, goal_state);
    siC_->nullControl(goal_motion->control);

    ROS_ERROR("happy happy, joy joy: %i", max_size);
    Motion* goal = nn_->nearest(goal_motion);
    Motion* motion = goal;
    Motion* parent;

    for(int i = 0; i < max_size * 0.2; ++i)
    {
//      ROS_ERROR("happy happy, joy joy %i: %i", i, motion);
//      if(motion == NULL)
//        break;
      parent = motion->parent;
      nn_->remove(motion);
    }

    delete goal_motion;
  }

protected:
  bool same_state(const ompl::base::SE2StateSpace::StateType* a, const ompl::base::SE2StateSpace::StateType* b)
  {
    double epsilon = 0.02;
    double epsilon_angle = 0.01;
    if (fabs(a->getX() - b->getX()) < epsilon && fabs(a->getY() - b->getY()) < epsilon
        && fabs(a->getYaw() - b->getYaw()) < epsilon_angle)
    {
      return true;
    }

    return false;
  }
};

}

#endif /* BACKWARDS_RRT_BACKWARDSRRT_H_ */
