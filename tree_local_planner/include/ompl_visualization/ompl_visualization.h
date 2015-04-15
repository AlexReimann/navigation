#ifndef OMPL_VISUALIZATION_OMPLVISUALIZATION_H_
#define OMPL_VISUALIZATION_OMPLVISUALIZATION_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ompl/base/PlannerData.h>

namespace ompl_visualization
{

class OmplVisualization
{
public:
  OmplVisualization();

  void visualize(const ompl::base::PlannerDataPtr planner_data, std::string frame_id);
  void visualize_path_states(std::vector<ompl::base::State *>& states, std::string frame_id, std::string path_namespace);
  void add_big_marker(std::string frame_id, int id, double x, double y, double z, float color_r, float color_g,
                      float color_b);

protected:
  visualization_msgs::Marker make_marker(std::string frame_id, int id, double x, double y, double z, std::string path_namespace);

  ros::Publisher marker_pub_;
  ros::Publisher marker_array_pub_;
};

} /* namespace ompl_visualization */

#endif /* OMPL_VISUALIZATION_OMPLVISUALIZATION_H_ */
