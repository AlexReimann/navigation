#include <costmap_2d/voxel_layer.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>

#define VOXEL_BITS 16
PLUGINLIB_EXPORT_CLASS(costmap_2d::VoxelLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void VoxelLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  ros::NodeHandle private_nh("~/" + name_);

  private_nh.param("publish_voxel_map", publish_voxel_, false);
  if (publish_voxel_)
    voxel_pub_ = private_nh.advertise < costmap_2d::VoxelGrid > ("voxel_grid", 1);

  clearing_endpoints_pub_ = private_nh.advertise<sensor_msgs::PointCloud>( "clearing_endpoints", 1 );
  cleared_points_pub_ = private_nh.advertise<sensor_msgs::PointCloud>( "cleared_points", 1 );

  combination_method_ = 1;

  private_nh.param("raytrace_corner_cases", raytrace_corner_cases_, false);
  private_nh.param("padded_raytracing", padded_raytracing_, false);
  private_nh.param("use_accurate_bresenham", use_accurate_bresenham_, false);
}

void VoxelLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::VoxelPluginConfig>::CallbackType cb = boost::bind(
      &VoxelLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

VoxelLayer::~VoxelLayer()
{
  if(dsrv_)
    delete dsrv_;
}

void VoxelLayer::reconfigureCB(costmap_2d::VoxelPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  size_z_ = config.z_voxels;
  origin_z_ = config.origin_z;
  z_resolution_ = config.z_resolution;
  unknown_threshold_ = config.unknown_threshold + (VOXEL_BITS - size_z_);
  mark_threshold_ = config.mark_threshold;
  combination_method_ = config.combination_method;
  matchSize();
}

void VoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
  ROS_ASSERT(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
}

void VoxelLayer::reset()
{
  deactivate();
  resetMaps();
  voxel_grid_.reset();
  activate();
}

void VoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                       double* min_y, double* max_x, double* max_y)
{
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  //get the marking observations
  current = current && getMarkingObservations(observations);

  //get the clearing observations
  current = current && getClearingObservations(clearing_observations);

  //update the global current status
  current_ = current;

  //raytrace freespace
  for (unsigned int i = 0; i < clearing_observations.size(); ++i)
  {
    raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
  }

  //place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      //if the obstacle is too high or too far away from the robot we won't add it
      if (cloud.points[i].z > max_obstacle_height_)
        continue;

      //compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (cloud.points[i].x - obs.origin_.x) * (cloud.points[i].x - obs.origin_.x)
          + (cloud.points[i].y - obs.origin_.y) * (cloud.points[i].y - obs.origin_.y)
          + (cloud.points[i].z - obs.origin_.z) * (cloud.points[i].z - obs.origin_.z);

      //if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range)
        continue;

      //now we need to compute the map coordinates for the observation
      unsigned int mx, my, mz;
      if (!worldToMap3D(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, mx, my, mz))
      {
        continue;
      }

      //mark the cell in the voxel grid and check if we should also mark it in the costmap
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_))
      {
        unsigned int index = getIndex(mx, my);

        costmap_[index] = LETHAL_OBSTACLE;
        touch((double)cloud.points[i].x, (double)cloud.points[i].y, min_x, min_y, max_x, max_y);
      }
    }
  }

  if (publish_voxel_)
  {
    costmap_2d::VoxelGrid grid_msg;
    unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid_msg.size_x = voxel_grid_.sizeX();
    grid_msg.size_y = voxel_grid_.sizeY();
    grid_msg.size_z = voxel_grid_.sizeZ();
    grid_msg.data.resize(size);
    memcpy(&grid_msg.data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    grid_msg.origin.x = origin_x_;
    grid_msg.origin.y = origin_y_;
    grid_msg.origin.z = origin_z_;

    grid_msg.resolutions.x = resolution_;
    grid_msg.resolutions.y = resolution_;
    grid_msg.resolutions.z = z_resolution_;
    grid_msg.header.frame_id = global_frame_;
    grid_msg.header.stamp = ros::Time::now();
    voxel_pub_.publish(grid_msg);
  }

  footprint_layer_.updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void VoxelLayer::clearNonLethal(double wx, double wy, double w_size_x, double w_size_y, bool clear_no_info)
{
  //get the cell coordinates of the center point of the window
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my))
    return;

  //compute the bounds of the window
  double start_x = wx - w_size_x / 2;
  double start_y = wy - w_size_y / 2;
  double end_x = start_x + w_size_x;
  double end_y = start_y + w_size_y;

  //scale the window based on the bounds of the costmap
  start_x = std::max(origin_x_, start_x);
  start_y = std::max(origin_y_, start_y);

  end_x = std::min(origin_x_ + getSizeInMetersX(), end_x);
  end_y = std::min(origin_y_ + getSizeInMetersY(), end_y);

  //get the map coordinates of the bounds of the window
  unsigned int map_sx, map_sy, map_ex, map_ey;

  //check for legality just in case
  if (!worldToMap(start_x, start_y, map_sx, map_sy) || !worldToMap(end_x, end_y, map_ex, map_ey))
    return;

  //we know that we want to clear all non-lethal obstacles in this window to get it ready for inflation
  unsigned int index = getIndex(map_sx, map_sy);
  unsigned char* current = &costmap_[index];
  for (unsigned int j = map_sy; j <= map_ey; ++j)
  {
    for (unsigned int i = map_sx; i <= map_ex; ++i)
    {
      //if the cell is a lethal obstacle... we'll keep it and queue it, otherwise... we'll clear it
      if (*current != LETHAL_OBSTACLE)
      {
        if (clear_no_info || *current != NO_INFORMATION)
        {
          *current = FREE_SPACE;
          voxel_grid_.clearVoxelColumn(index);
        }
      }
      current++;
      index++;
    }
    current += size_x_ - (map_ex - map_sx) - 1;
    index += size_x_ - (map_ex - map_sx) - 1;
  }
}

void VoxelLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                           double* max_x, double* max_y)
{
  if (clearing_observation.cloud_->points.size() == 0)
    return;

  double sensor_x, sensor_y, sensor_z;
  double ox = clearing_observation.origin_.x;
  double oy = clearing_observation.origin_.y;
  double oz = clearing_observation.origin_.z;

  if (!worldToMap3DFloat(ox, oy, oz, sensor_x, sensor_y, sensor_z))
  {
    ROS_WARN_THROTTLE(
        1.0,
        "The origin for the sensor at (%.2f, %.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
        ox, oy, oz);
    return;
  }

  bool publish_clearing_points = (clearing_endpoints_pub_.getNumSubscribers() > 0);
  if( publish_clearing_points )
  {
    clearing_endpoints_.points.clear();
    clearing_endpoints_.points.reserve( clearing_observation.cloud_->points.size() );
  }

  unsigned int padding_size = 1; //to handled padded raytracing
  unsigned int update_area_center = floor(max_raytrace_range_ / resolution_) + padding_size;
  unsigned int updated_area_width = floor(max_raytrace_range_ / resolution_) * 2 + 1 + 2 * padding_size;
  unsigned int max_updated_area_size = pow(updated_area_width, 2); // | padding | raytrace | center | raytrace | padding

  double temp = 0;

  //need to add the residual to have the same sub-cell-accuracy starting offset for Bresenham algorithm
  double offset_x = update_area_center + fabs(modf(sensor_x, &temp));
  double offset_y = update_area_center + fabs(modf(sensor_y, &temp));

  boost::shared_ptr<uint32_t[]> padded_voxel_grid_mask(new uint32_t[max_updated_area_size]);
  uint32_t empty_mask = (uint32_t)0;

  for (int i = 0; i < max_updated_area_size; ++i)
    padded_voxel_grid_mask[i] = empty_mask;

  boost::shared_ptr<bool[]> updatedColumns(new bool[max_updated_area_size]);

  for (int i = 0; i < max_updated_area_size; ++i)
    updatedColumns[i] = false;

  //we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
  double map_end_x = origin_x_ + getSizeInMetersX();
  double map_end_y = origin_y_ + getSizeInMetersY();

  for (unsigned int i = 0; i < clearing_observation.cloud_->points.size(); ++i)
  {
    double wpx = clearing_observation.cloud_->points[i].x;
    double wpy = clearing_observation.cloud_->points[i].y;
    double wpz = clearing_observation.cloud_->points[i].z;

    double dx = wpx - ox;
    double dy = wpy - oy;
    double dz = wpz - oz;
    double scaling = 1.0;

    //we can only raytrace to a maximum z height
    if (wpz > max_obstacle_height_)
    {
      //we know we want the vector's z value to be max_z
      scaling = std::max(0.0, std::min(scaling, (max_obstacle_height_ - oz) / dz));
    }
    //and we can only raytrace down to the floor
    else if (wpz < origin_z_)
    {
      //we know we want the vector's z value to be 0.0
      scaling = std::min(scaling, (origin_z_ - oz) / dz);
    }

    //the minimum value to raytrace from is the origin
    if (wpx < origin_x_)
    {
      scaling = std::min(scaling, (origin_x_ - ox) / dx);
    }
    if (wpy < origin_y_)
    {
      scaling = std::min(scaling, (origin_y_ - oy) / dy);
    }

    //the maximum value to raytrace to is the end of the map
    if (wpx > map_end_x)
    {
      scaling = std::min(scaling, (map_end_x - ox) / dx);
    }
    if (wpy > map_end_y)
    {
      scaling = std::min(scaling, (map_end_y - oy) / dy);
    }

    if (dx * dx + dy * dy + dz * dz > clearing_observation.raytrace_range_ * clearing_observation.raytrace_range_)
    {
      scaling = std::min(scaling, clearing_observation.raytrace_range_ / sqrt(dx * dx + dy * dy + dz * dz));
    }

    wpx = ox + dx * scaling;
    wpy = oy + dy * scaling;
    wpz = oz + dz * scaling;

    double point_x, point_y, point_z;
    if (worldToMap3DFloat(wpx, wpy, wpz, point_x, point_y, point_z))
    {
      unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);

      if (use_accurate_bresenham_)
      {
        voxel_grid_.updateClearingMaskNew(padded_voxel_grid_mask, updatedColumns, updated_area_width, offset_x,
                                          offset_y, sensor_z, offset_x + (point_x - sensor_x),
                                          offset_y + (point_y - sensor_y), point_z, cell_raytrace_range);
      }
      else
      {
        voxel_grid_.updateClearingMask(padded_voxel_grid_mask, updatedColumns, updated_area_width, update_area_center,
                                       update_area_center, sensor_z, update_area_center + point_x - (int)sensor_x,
                                       update_area_center + point_y - (int)sensor_y, point_z, cell_raytrace_range,
                                       raytrace_corner_cases_, padded_raytracing_);
      }

      updateRaytraceBounds(ox, oy, wpx, wpy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);

      if( publish_clearing_points )
      {
        geometry_msgs::Point32 point;
        point.x = wpx;
        point.y = wpy;
        point.z = wpz;
        clearing_endpoints_.points.push_back( point );
      }
    }
  }

  voxel_grid_.updateGrid(padded_voxel_grid_mask, updated_area_width, (int)sensor_x - update_area_center, (int)sensor_y - update_area_center);
  voxel_grid_.updateCostmap(costmap_, updatedColumns, updated_area_width, (int)sensor_x - update_area_center, (int)sensor_y - update_area_center, unknown_threshold_, mark_threshold_, FREE_SPACE, NO_INFORMATION);

  setUpdatedCells(updatedColumns, updated_area_width, (int)sensor_x - update_area_center, (int)sensor_y - update_area_center);

  voxel_grid_.updateGrid(padded_voxel_grid_mask, updated_area_width, (int)sensor_x - update_area_center, (int)sensor_y - update_area_center);

  bool publish_cleared_points = (cleared_points_pub_.getNumSubscribers() > 0);

  if(publish_cleared_points)
    publishClearedCells(padded_voxel_grid_mask, updated_area_width, (int)sensor_x - update_area_center, (int)sensor_y - update_area_center, sensor_z);

  if( publish_clearing_points )
  {
    clearing_endpoints_.header.frame_id = global_frame_;
    clearing_endpoints_.header.stamp = pcl_conversions::fromPCL(clearing_observation.cloud_->header).stamp;
    clearing_endpoints_.header.seq = clearing_observation.cloud_->header.seq;

    clearing_endpoints_pub_.publish( clearing_endpoints_ );
  }
}

void VoxelLayer::setUpdatedCells(boost::shared_ptr<bool[]> updated_columns, unsigned int updated_area_width, unsigned int offset_x, unsigned int offset_y)
{
  updated_cells_index_.clear();

  unsigned int linear_index = 0;
  unsigned int linear_index_offset = 0;
  int x = 0;
  int y = 0;

  for (unsigned int row_index = 0; row_index < updated_area_width; ++row_index)
  {
    linear_index = row_index * updated_area_width;
    y = row_index + offset_y;
    if (y < 0)
      continue;

    if (y >= size_y_)
      break;

    x = offset_x;

    linear_index_offset = y * size_x_ + x;
    for (unsigned int column_index = 0; column_index < updated_area_width; ++column_index)
    {
      if (!updated_columns[linear_index] || x < 0)
      {
        x++;
        linear_index++;
        linear_index_offset++;
        continue;
      }

      if (x >= size_x_)
        break;

      MapLocation cell_location;
      cell_location.x = x;
      cell_location.y = y;

      updated_cells_index_.push_back(cell_location);

      x++;
      linear_index_offset++;
      linear_index++;
    }
  }
}

void VoxelLayer::publishClearedCells(boost::shared_ptr<uint32_t[]>& grid_mask, unsigned int updated_area_width, int offset_x, int offset_y, double sensor_z)
{
  uint32_t updateIndexMask;
  sensor_msgs::PointCloud cleared_points;

  cleared_points.points.reserve(updated_cells_index_.size());

  unsigned int linear_index = 0;
  unsigned int linear_index_offset = 0;

  double max_map = 0;
  double max_world = 0;

  int x = 0;
  int y = 0;

  for (unsigned int row_index = 0; row_index < updated_area_width; ++row_index)
  {
    linear_index = row_index * updated_area_width;
    y = row_index + offset_y;

    if (y < 0)
      continue;

    if (y >= size_y_)
      break;

    x = offset_x;

    linear_index_offset = y * size_x_ + x;

    for (unsigned int column_index = 0; column_index < updated_area_width; ++column_index)
    {
      if (x < 0)
      {
        x++;
        linear_index_offset++;
        linear_index++;
        continue;
      }

      if (x >= size_x_)
        break;

      updateIndexMask = grid_mask[linear_index] >> 1;

      for (int z_index = 0; z_index < 16; ++z_index)
      {
        if ((updateIndexMask & 1) != 0)
        {
          double point_x, point_y, point_z;

          mapToWorld3D(x, y, z_index, point_x, point_y, point_z);

          geometry_msgs::Point32 point;
          point.x = point_x;
          point.y = point_y;
          point.z = point_z;
          cleared_points.points.push_back(point);

          if(max_world < point_z)
          {
            max_world = point_z;
          }

        }
        updateIndexMask >>= 1;
      }

      x++;
      linear_index_offset++;
      linear_index++;
    }
  }

  cleared_points.header.frame_id = global_frame_;
  cleared_points.header.stamp = ros::Time::now();
  cleared_points_pub_.publish(cleared_points);
}

void VoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  //project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  //compute the associated world coordinates for the origin cell
  //beacuase we want to keep things grid-aligned
  double new_grid_ox, new_grid_oy;
  new_grid_ox = origin_x_ + cell_ox * resolution_;
  new_grid_oy = origin_y_ + cell_oy * resolution_;

  //To save casting from unsigned int to int a bunch of times
  int size_x = size_x_;
  int size_y = size_y_;

  //we need to compute the overlap of the new and existing windows
  int lower_left_x, lower_left_y, upper_right_x, upper_right_y;
  lower_left_x = std::min(std::max(cell_ox, 0), size_x);
  lower_left_y = std::min(std::max(cell_oy, 0), size_y);
  upper_right_x = std::min(std::max(cell_ox + size_x, 0), size_x);
  upper_right_y = std::min(std::max(cell_oy + size_y, 0), size_y);

  unsigned int cell_size_x = upper_right_x - lower_left_x;
  unsigned int cell_size_y = upper_right_y - lower_left_y;

  //we need a map to store the obstacles in the window temporarily
  unsigned char* local_map = new unsigned char[cell_size_x * cell_size_y];
  unsigned int* local_voxel_map = new unsigned int[cell_size_x * cell_size_y];
  unsigned int* voxel_map = voxel_grid_.getData();

  //copy the local window in the costmap to the local map
  copyMapRegion(costmap_, lower_left_x, lower_left_y, size_x_, local_map, 0, 0, cell_size_x, cell_size_x, cell_size_y);
  copyMapRegion(voxel_map, lower_left_x, lower_left_y, size_x_, local_voxel_map, 0, 0, cell_size_x, cell_size_x,
                cell_size_y);

  //we'll reset our maps to unknown space if appropriate
  resetMaps();

  //update the origin with the appropriate world coordinates
  origin_x_ = new_grid_ox;
  origin_y_ = new_grid_oy;

  //compute the starting cell location for copying data back in
  int start_x = lower_left_x - cell_ox;
  int start_y = lower_left_y - cell_oy;

  //now we want to copy the overlapping information back into the map, but in its new location
  copyMapRegion(local_map, 0, 0, cell_size_x, costmap_, start_x, start_y, size_x_, cell_size_x, cell_size_y);
  copyMapRegion(local_voxel_map, 0, 0, cell_size_x, voxel_map, start_x, start_y, size_x_, cell_size_x, cell_size_y);

  //make sure to clean up
  delete[] local_map;
  delete[] local_voxel_map;

}

}
