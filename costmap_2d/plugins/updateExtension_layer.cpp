#include<costmap_2d/updateExtension_layer.h>
#include<costmap_2d/footprint.h>
#include<string>
#include<sstream>
#include <cstdio>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(costmap_2d::UpdateExtensionLayer, costmap_2d::Layer)

namespace costmap_2d
{

  void UpdateExtensionLayer::initMaps(unsigned int size_x, unsigned int size_y)
  {
    persistentMap_ = new unsigned char[size_x * size_y];
    VoxelLayer::initMaps(size_x, size_y);
  }

  void UpdateExtensionLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
  {
    if(!enabled_) return;

    //update costmap_
    VoxelLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

    for (int y = *min_y; y < *max_y; y++)
    {
      unsigned int it = y * size_y_ + *min_x;

      for (int i = *min_x; i < *max_x; i++)
      {
        if (costmap_[it] != persistentMap_[it])
        {
          persistentMap_[it] = costmap_[it];
          continue;
        }

        //don't update things that didn't change
        costmap_[it] == NO_INFORMATION;
      }
    }
  }
}

