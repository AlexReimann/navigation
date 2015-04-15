#include <ompl_visualization/ompl_visualization.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ros/console.h>

namespace ompl_visualization
{

OmplVisualization::OmplVisualization()
{
  ros::NodeHandle nh;
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
}

void OmplVisualization::visualize(const ompl::base::PlannerDataPtr planner_data, std::string frame_id)
{
  ROS_INFO("planner_data->numVertices(): %i", planner_data->numVertices());

  visualization_msgs::MarkerArray marker_array;
  unsigned int vertex_id = 0;

  for (; vertex_id < planner_data->numVertices(); ++vertex_id)
  {
    const ompl::base::State *state = planner_data->getVertex(vertex_id).getState();
    const ompl::base::SE2StateSpace::StateType *casted_state =
        static_cast<const ompl::base::SE2StateSpace::StateType*>(state);
    marker_array.markers.push_back(make_marker(frame_id, vertex_id, casted_state->getX(), casted_state->getY(), 0));
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = "ompl";
  marker.action = visualization_msgs::Marker::DELETE;

//  for (; vertex_id < 5000; ++vertex_id)
//  {
//    marker.id = vertex_id;
//    marker_array.markers.push_back(marker);
//  }
//
//  marker_array_pub_.publish(marker_array);
}

void OmplVisualization::visualize_path_states(std::vector<ompl::base::State*>& states, std::string frame_id)
{
  visualization_msgs::MarkerArray marker_array;
  unsigned int vertex_id = 0;

  for (unsigned int i; i < states.size(); ++i)
  {
    const ompl::base::SE2StateSpace::StateType *casted_state =
        static_cast<const ompl::base::SE2StateSpace::StateType*>(states[i]);
    marker_array.markers.push_back(make_marker(frame_id, i, casted_state->getX(), casted_state->getY(), 0));
  }

//  visualization_msgs::Marker marker;
//  marker.header.frame_id = frame_id;
//  marker.header.stamp = ros::Time::now();
//
//  marker.ns = "ompl";
//  marker.action = visualization_msgs::Marker::DELETE;
//
//  for (; vertex_id < 200; ++vertex_id)
//  {
//    marker.id = vertex_id;
//    marker_array.markers.push_back(marker);
//  }

  marker_array_pub_.publish(marker_array);
}

visualization_msgs::Marker OmplVisualization::make_marker(std::string frame_id, int id, double x, double y, double z)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = "ompl";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0.1);

  double marker_scaling = 0.05;

  marker.scale.x = marker_scaling;
  marker.scale.y = marker_scaling;
  marker.scale.z = marker_scaling;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  return marker;
}

void OmplVisualization::add_big_marker(std::string frame_id, int id, double x, double y, double z, float color_r,
                                       float color_g, float color_b)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = "ompl_big";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0.1);

  double marker_scaling = 0.2;

  marker.scale.x = marker_scaling;
  marker.scale.y = marker_scaling;
  marker.scale.z = marker_scaling;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.color.r = color_r;
  marker.color.g = color_g;
  marker.color.b = color_b;
  marker.color.a = 1.0;

  marker_pub_.publish(marker);
}

} /* namespace ompl_visualization */
