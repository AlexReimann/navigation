#ifndef BACKWARDS_RRT_BACKWARDSRRT_H_
#define BACKWARDS_RRT_BACKWARDSRRT_H_

#include <ompl/control/planners/rrt/RRT.h>

namespace backward_rrt
{

class BackwardRRT : public ompl::control::RRT
{

public:
  BackwardRRT(const ompl::control::SpaceInformationPtr &si) : ompl::control::RRT(si)
  {

  }

  void clear()
  {
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    lastGoalMotion_ = NULL;
  }
};

}

#endif /* BACKWARDS_RRT_BACKWARDSRRT_H_ */
