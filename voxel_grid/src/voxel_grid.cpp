/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <voxel_grid/voxel_grid.h>
#include <sys/time.h>
#include <bitset>
#include <ros/console.h>

namespace voxel_grid {
  VoxelGrid::VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z)
  {
    size_x_ = size_x; 
    size_y_ = size_y; 
    size_z_ = size_z; 

    if(size_z_ > 16){
      ROS_INFO("Error, this implementation can only support up to 16 z values (%d)", size_z_); 
      size_z_ = 16;
    }

    data_ = new uint32_t[size_x_ * size_y_];
    uint32_t unknown_col = ~((uint32_t)0)>>16;
    uint32_t* col = data_;
    for(unsigned int i = 0; i < size_x_ * size_y_; ++i){
      *col = unknown_col;
      ++col;
    }
  }

  void VoxelGrid::resize(unsigned int size_x, unsigned int size_y, unsigned int size_z)
  {
    //if we're not actually changing the size, we can just reset things
    if(size_x == size_x_ && size_y == size_y_ && size_z == size_z_){
      reset();
      return;
    }

    delete[] data_;
    size_x_ = size_x; 
    size_y_ = size_y; 
    size_z_ = size_z; 

    if(size_z_ > 16){
      ROS_INFO("Error, this implementation can only support up to 16 z values (%d)", size_z); 
      size_z_ = 16;
    }

    data_ = new uint32_t[size_x_ * size_y_];
    uint32_t unknown_col = ~((uint32_t)0)>>16;
    uint32_t* col = data_;
    for(unsigned int i = 0; i < size_x_ * size_y_; ++i){
      *col = unknown_col;
      ++col;
    }
  }

  VoxelGrid::~VoxelGrid()
  {
    delete [] data_;
  }

  void VoxelGrid::reset(){
    uint32_t unknown_col = ~((uint32_t)0)>>16;
    uint32_t* col = data_;
    for(unsigned int i = 0; i < size_x_ * size_y_; ++i){
      *col = unknown_col;
      ++col;
    }
  }

  void VoxelGrid::markVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length){
    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1, 
          size_x_, size_y_, size_z_);
      return;
    }

    MarkVoxel mv(data_);
    raytraceLine(mv, x0, y0, z0, x1, y1, z1, max_length);
  }

  void VoxelGrid::clearVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length){
    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1, 
          size_x_, size_y_, size_z_);
      return;
    }

    ClearVoxel cv(data_);
    raytraceLine(cv, x0, y0, z0, x1, y1, z1, max_length);
  }

  void VoxelGrid::clearVoxelLineInMap(double x0, double y0, double z0, double x1, double y1, double z1, unsigned char *map_2d, 
      unsigned int unknown_threshold, unsigned int mark_threshold, unsigned char free_cost, unsigned char unknown_cost, unsigned int max_length){
    costmap = map_2d;
    if(map_2d == NULL){
      clearVoxelLine(x0, y0, z0, x1, y1, z1, max_length);
      return;
    }

    if(x0 >= size_x_ || y0 >= size_y_ || z0 >= size_z_ || x1>=size_x_ || y1>=size_y_ || z1>=size_z_){
      ROS_DEBUG("Error, line endpoint out of bounds. (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f),  size: (%d, %d, %d)", x0, y0, z0, x1, y1, z1, 
          size_x_, size_y_, size_z_);
      return;
    }

    ClearVoxelInMap cvm(data_, costmap, unknown_threshold, mark_threshold, free_cost, unknown_cost);
    raytraceLine(cvm, x0, y0, z0, x1, y1, z1, max_length);
  }

  void VoxelGrid::updateClearingMask(boost::shared_ptr<uint32_t[]>& grid_mask,
                                     boost::shared_ptr<bool[]>& updated_columns, unsigned int updated_area_width,
                                     double x0, double y0, double z0, double x1, double y1, double z1,
                                     unsigned int max_length, bool raytrace_corner_cases, bool padded_raytracing)
  {
    int dx = int(x1) - int(x0);
    int dy = int(y1) - int(y0);
    int dz = int(z1) - int(z0);

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    unsigned int abs_dz = abs(dz);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * updated_area_width;
    int offset_dz = sign(dz);

    unsigned int z_mask = 1 << (unsigned int)(z0 + 1); //+1 because of padding
    unsigned int offset = (unsigned int)y0 * updated_area_width + (unsigned int)x0;

    GridOffset grid_off(offset);
    ZOffset z_off(z_mask);

    //we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
    double scale = std::min(1.0, max_length / dist);

    //is x dominant
    if (abs_dx >= max(abs_dy, abs_dz))
    {
      int error_y = abs_dx / 2;
      int error_z = abs_dx / 2;

      if (!raytrace_corner_cases)
      {
        if (padded_raytracing)
        {
          bresenham3DOriginal(GridMaskUpdaterX(grid_mask, updated_columns, updated_area_width), grid_off, grid_off, z_off,
                              abs_dx, abs_dy, abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask,
                              (unsigned int)(scale * abs_dx));
          return;
        }

        bresenham3DOriginal(GridMaskUpdater(grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dx, abs_dy,
                            abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask,
                            (unsigned int)(scale * abs_dx));
        return;
      }

      if (padded_raytracing)
      {
        bresenham3D(GridMaskUpdaterX(grid_mask, updated_columns, updated_area_width), grid_off, grid_off, z_off, abs_dx,
                    abs_dy, abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask,
                    (unsigned int)(scale * abs_dx));
        return;
      }

      bresenham3D(GridMaskUpdater(grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dx, abs_dy, abs_dz,
                  error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask, (unsigned int)(scale * abs_dx));
      return;
    }

    //y is dominant
    if (abs_dy >= abs_dz)
    {
      int error_x = abs_dy / 2;
      int error_z = abs_dy / 2;

      if (!raytrace_corner_cases)
      {
        if (padded_raytracing)
        {
          bresenham3DOriginal(GridMaskUpdaterY(grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dy,
                              abs_dx, abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask,
                              (unsigned int)(scale * abs_dy));
          return;
        }

        bresenham3DOriginal(GridMaskUpdater(grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dy, abs_dx,
                            abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask,
                            (unsigned int)(scale * abs_dy));
        return;
      }

      if (padded_raytracing)
      {
        bresenham3D(GridMaskUpdaterY(grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dy, abs_dx,
                    abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask,
                    (unsigned int)(scale * abs_dy));
        return;
      }

      bresenham3D(GridMaskUpdater(grid_mask, updated_columns), grid_off, grid_off, z_off, abs_dy, abs_dx, abs_dz,
                  error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask, (unsigned int)(scale * abs_dy));
      return;
    }

    //otherwise, z is dominant
    int error_x = abs_dz / 2;
    int error_y = abs_dz / 2;

    if (!raytrace_corner_cases)
    {
      if (padded_raytracing)
      {
        bresenham3DOriginal(GridMaskUpdaterZ(grid_mask, updated_columns, updated_area_width), z_off, grid_off, grid_off,
                            abs_dz, abs_dx, abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask,
                            (unsigned int)(scale * abs_dz));
        return;
      }

      bresenham3DOriginal(GridMaskUpdater(grid_mask, updated_columns), z_off, grid_off, grid_off, abs_dz, abs_dx,
                          abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask,
                          (unsigned int)(scale * abs_dz));
      return;
    }

    if (padded_raytracing)
    {
      bresenham3D(GridMaskUpdaterZ(grid_mask, updated_columns, updated_area_width), z_off, grid_off, grid_off, abs_dz,
                  abs_dx, abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask,
                  (unsigned int)(scale * abs_dz));
      return;
    }

    bresenham3D(GridMaskUpdater(grid_mask, updated_columns), z_off, grid_off, grid_off, abs_dz, abs_dx, abs_dy,
                error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask, (unsigned int)(scale * abs_dz));
    return;
  }

  void VoxelGrid::updateClearingMaskNew(boost::shared_ptr<uint32_t[]>& grid_mask,
                                        boost::shared_ptr<bool[]>& updated_columns, unsigned int updated_area_width,
                                        double x0, double y0, double z0, double x1, double y1, double z1,
                                        unsigned int max_length)
  {
    double scaling_amount = pow(2, 16); //more is more accurate but lower max value (as we have limit of MAX_INT)

    double dx = x1 - x0;
    double dy = y1 - y0;
    double dz = z1 - z0;

    double abs_dx = fabs(scaling_amount * dx);
    double abs_dy = fabs(scaling_amount * dy);
    double abs_dz = fabs(scaling_amount * dz);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * updated_area_width;
    int offset_dz = sign(dz);

    unsigned int z_mask = 1 << (unsigned int)(z0 + 1); //+1 because of padding, is undone later
    unsigned int offset = (unsigned int)y0 * updated_area_width + (unsigned int)x0;

    GridOffset index_updater_xy(offset);
    ZOffset index_updater_z(z_mask);

    double temp = 0;
    double x_lost_rounding = dx > 0 ? 1.0 - fabs(modf(x0, &temp)) : fabs(modf(x0, &temp));
    double y_lost_rounding = dy > 0 ? 1.0 - fabs(modf(y0, &temp)) : fabs(modf(y0, &temp));
    double z_lost_rounding = dz > 0 ? 1.0 - fabs(modf(z0, &temp)) : fabs(modf(z0, &temp));

    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
    double scale = std::min(1.0, max_length / dist);

    if (abs_dx >= max(abs_dy, abs_dz))
    {
      int error_y = (int)(abs_dy * x_lost_rounding - abs_dx * y_lost_rounding);
      int error_z = (int)(abs_dz * x_lost_rounding - abs_dx * z_lost_rounding);

      bresenham3Dnew(GridMaskUpdater(grid_mask, updated_columns), index_updater_xy, index_updater_xy, index_updater_z,
                     abs_dx, abs_dy, abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask,
                     abs(int(x0) - int(x1)), (unsigned int)(scale * abs(int(x0) - int(x1))));
      return;
    }

    if (abs_dy >= abs_dz)
    {
      int error_x = (int)(abs_dx * y_lost_rounding - abs_dy * x_lost_rounding);
      int error_z = (int)(abs_dz * y_lost_rounding - abs_dy * z_lost_rounding);

      bresenham3Dnew(GridMaskUpdater(grid_mask, updated_columns), index_updater_xy, index_updater_xy, index_updater_z,
                     abs_dy, abs_dx, abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask,
                     abs(int(y0) - int(y1)), (unsigned int)(scale * abs(int(y0) - int(y1))));
      return;
    }

    int error_x = (int)(abs_dx * z_lost_rounding - abs_dz * x_lost_rounding);
    int error_y = (int)(abs_dy * z_lost_rounding - abs_dz * y_lost_rounding);

    bresenham3Dnew(GridMaskUpdater(grid_mask, updated_columns), index_updater_z, index_updater_xy, index_updater_xy,
                   abs_dz, abs_dx, abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask,
                   abs(int(z0) - int(z1)), (unsigned int)(scale * abs(int(z0) - int(z1))));
  }

  void VoxelGrid::updateGrid(boost::shared_ptr<uint32_t[]>& grid_mask, unsigned int updated_area_width, int offset_x,
                             int offset_y)
  {
    uint32_t zMask;

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
        if (x < 0)
        {
          x++;
          linear_index_offset++;
          linear_index++;
          continue;
        }

        if (x >= size_x_)
          break;

        zMask = convertUpdateMaskToZMask(grid_mask[linear_index]);
        data_[linear_index_offset] &= ~(zMask);

        x++;
        linear_index_offset++;
        linear_index++;
      }
    }
  }

  void VoxelGrid::updateCostmap(unsigned char* costmap, boost::shared_ptr<bool[]>& updated_columns,
                                unsigned int updated_area_width, int offset_x, int offset_y,
                                unsigned int unknown_clear_threshold, unsigned int marked_clear_threshold,
                                unsigned char free_cost, unsigned char unknown_cost)
  {
    unsigned int linear_index = 0;
    unsigned int linear_index_offset = 0;
    int x = 0;
    int y = 0;

    for (unsigned int row_index = 0; row_index < updated_area_width; ++row_index)
    {
      linear_index = row_index * updated_area_width;
      y = row_index + offset_y;

      if(y < 0)
        continue;

      if(y >= size_y_)
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

        uint32_t column_occupation = data_[linear_index_offset];

        unsigned int unknown_bits = uint16_t(column_occupation >> 16) ^ uint16_t(column_occupation);
        unsigned int marked_bits = column_occupation >> 16;

        if (bitsBelowThreshold(marked_bits, marked_clear_threshold))
        {
          if (bitsBelowThreshold(unknown_bits, unknown_clear_threshold))
          {
            costmap[linear_index_offset] = free_cost;
          }
          else
          {
            costmap[linear_index_offset] = unknown_cost;
          }
        }

        x++;
        linear_index_offset++;
        linear_index++;
      }
    }
  }

  VoxelStatus VoxelGrid::getVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if(x >= size_x_ || y >= size_y_ || z >= size_z_){
      ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
      return UNKNOWN;
    }
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    uint32_t result = data_[y * size_x_ + x] & full_mask; 
    unsigned int bits = numBits(result);

    // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
    if(bits < 2){
      if(bits < 1)
        return FREE;

      return UNKNOWN;
    }

    return MARKED;
  }

  VoxelStatus VoxelGrid::getVoxelColumn(unsigned int x, unsigned int y, unsigned int unknown_threshold, unsigned int marked_threshold)
  {
    if(x >= size_x_ || y >= size_y_){
      ROS_DEBUG("Error, voxel out of bounds. (%d, %d)\n", x, y);
      return UNKNOWN;
    }
    
    uint32_t* col = &data_[y * size_x_ + x];

    unsigned int unknown_bits = uint16_t(*col>>16) ^ uint16_t(*col);
    unsigned int marked_bits = *col>>16;

    //check if the number of marked bits qualifies the col as marked
    if(!bitsBelowThreshold(marked_bits, marked_threshold)){
      return MARKED;
    }

    //check if the number of unkown bits qualifies the col as unknown
    if(!bitsBelowThreshold(unknown_bits, unknown_threshold))
      return UNKNOWN;

    return FREE;
  }

  unsigned int VoxelGrid::sizeX(){
    return size_x_;
  }

  unsigned int VoxelGrid::sizeY(){
    return size_y_;
  }

  unsigned int VoxelGrid::sizeZ(){
    return size_z_;
  }

  void VoxelGrid::printVoxelGrid(){
    for(unsigned int z = 0; z < size_z_; z++){
      printf("Layer z = %u:\n",z);
      for(unsigned int y = 0; y < size_y_; y++){
        for(unsigned int x = 0 ; x < size_x_; x++){
          printf((getVoxel(x, y, z)) == voxel_grid::MARKED? "#" : " ");
        }
        printf("|\n");
      } 
    }
  }

  void VoxelGrid::printColumnGrid(){
    printf("Column view:\n");
    for(unsigned int y = 0; y < size_y_; y++){
      for(unsigned int x = 0 ; x < size_x_; x++){
        printf((getVoxelColumn(x, y, 16, 0) == voxel_grid::MARKED)? "#" : " ");
      }
      printf("|\n");
    } 
  }
};
