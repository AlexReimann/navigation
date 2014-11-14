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
#ifndef VOXEL_GRID_VOXEL_GRID_H
#define VOXEL_GRID_VOXEL_GRID_H

#include <cmath>
#include <limits.h>
#include <algorithm>
#include <ros/console.h>
#include <ros/assert.h>
#include <bitset>

/**
 * @class VoxelGrid
 * @brief A 3D grid structure that stores points as an integer array.
 *        X and Y index the array and Z selects which bit of the integer
 *        is used giving a limit of 16 vertical cells.
 */
namespace voxel_grid
{

enum VoxelStatus {
  FREE = 0,
  UNKNOWN = 1,
  MARKED = 2,
};

class VoxelGrid
{
public:
  /**
   * @brief  Constructor for a voxel grid
   * @param size_x The x size of the grid
   * @param size_y The y size of the grid
   * @param size_z The z size of the grid, only sizes <= 16 are supported
   */
  VoxelGrid(unsigned int size_x, unsigned int size_y, unsigned int size_z);

  ~VoxelGrid();

  /**
   * @brief  Resizes a voxel grid to the desired size
   * @param size_x The x size of the grid
   * @param size_y The y size of the grid
   * @param size_z The z size of the grid, only sizes <= 16 are supported
   */
  void resize(unsigned int size_x, unsigned int size_y, unsigned int size_z);

  void reset();
  uint32_t* getData() { return data_; }

  inline void markVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return;
    }
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    data_[y * size_x_ + x] |= full_mask; //clear unknown and mark cell
  }

  inline bool markVoxelInMap(unsigned int x, unsigned int y, unsigned int z, unsigned int marked_threshold)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return false;
    }

    int index = y * size_x_ + x;
    uint32_t* col = &data_[index];
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    *col |= full_mask; //clear unknown and mark cell

    unsigned int marked_bits = *col>>16;

    //make sure the number of bits in each is below our thesholds
    return !bitsBelowThreshold(marked_bits, marked_threshold);
  }

  inline void clearVoxel(unsigned int x, unsigned int y, unsigned int z)
  {
    if (x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return;
    }
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    data_[y * size_x_ + x] &= ~(full_mask); //clear unknown and clear cell
  }

  inline void clearVoxelColumn(unsigned int index)
  {
    ROS_ASSERT(index < size_x_ * size_y_);
    data_[index] = 0;
  }

  inline void clearVoxelInMap(unsigned int x, unsigned int y, unsigned int z)
  {
    if(x >= size_x_ || y >= size_y_ || z >= size_z_)
    {
      ROS_DEBUG("Error, voxel out of bounds.\n");
      return;
    }
    int index = y * size_x_ + x;
    uint32_t* col = &data_[index];
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    *col &= ~(full_mask); //clear unknown and clear cell

    unsigned int unknown_bits = uint16_t(*col>>16) ^ uint16_t(*col);
    unsigned int marked_bits = *col>>16;

    //make sure the number of bits in each is below our thesholds
    if (bitsBelowThreshold(unknown_bits, 1) && bitsBelowThreshold(marked_bits, 1))
    {
      costmap_[index] = 0;
    }
  }

  inline bool bitsBelowThreshold(unsigned int n, unsigned int bit_threshold)
  {
    unsigned int bit_count;
    for (bit_count = 0; n;)
    {
      ++bit_count;
      if (bit_count > bit_threshold)
      {
        return false;
      }
      n &= n - 1; //clear the least significant bit set
    }
    return true;
  }

  static inline unsigned int numBits(unsigned int n)
  {
    unsigned int bit_count;
    for (bit_count = 0; n; ++bit_count)
    {
      n &= n - 1; //clear the least significant bit set
    }
    return bit_count;
  }

  static VoxelStatus getVoxel(
    unsigned int x, unsigned int y, unsigned int z,
    unsigned int size_x, unsigned int size_y, unsigned int size_z, const uint32_t* data)
  {
    if (x >= size_x || y >= size_y || z >= size_z)
    {
      ROS_DEBUG("Error, voxel out of bounds. (%d, %d, %d)\n", x, y, z);
      return UNKNOWN;
    }
    uint32_t full_mask = ((uint32_t)1<<z<<16) | (1<<z);
    uint32_t result = data[y * size_x + x] & full_mask;
    unsigned int bits = numBits(result);

    // known marked: 11 = 2 bits, unknown: 01 = 1 bit, known free: 00 = 0 bits
    if (bits < 2)
    {
      if (bits < 1)
      {
        return FREE;
      }
      return UNKNOWN;
    }
    return MARKED;
  }

  void markVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length = UINT_MAX);
  void clearVoxelLine(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length = UINT_MAX);
  void clearVoxelLineInMap(double x0, double y0, double z0, double x1, double y1, double z1, unsigned char *map_2d,
                           unsigned int unknown_threshold, unsigned int mark_threshold,
                           unsigned char free_cost = 0, unsigned char unknown_cost = 255, unsigned int max_length = UINT_MAX);
  void update(double x0, double y0, double z0, double x1, double y1, double z1, unsigned int max_length,
              bool* updated_columns);
  void updatePadded(uint32_t* padded_data, bool* updated_columns, double x0, double y0, double z0, double x1, double y1,
                    double z1, unsigned int max_length = UINT_MAX, unsigned int padding_size = 0);

  VoxelStatus getVoxel(unsigned int x, unsigned int y, unsigned int z);

  //Are there any obstacles at that (x, y) location in the grid?
  VoxelStatus getVoxelColumn(unsigned int x, unsigned int y,
                             unsigned int unknown_threshold = 0, unsigned int marked_threshold = 0);

  void printMask(unsigned int mask)
  {
    std::bitset<32> x(mask);
    std::cout << x << std::endl;
  }

  void updateGrid(uint32_t* padded_grid_mask, unsigned int padding_size = 0)
  {
    uint32_t mask;

    //if only one of this bits is set, this means we are not clearing it
    //so this means underflow or overflow happened
    uint32_t mask_overflow_test = ((unsigned int)1 << 16) | (unsigned int)1;
    uint32_t mask_underflow_test = ((unsigned int)1 << 31) | ((unsigned int)1 << 15);

    uint32_t mask_lower_bits =  ~((uint32_t)0) >> 16;

    unsigned int linear_index = 0;
    unsigned int linear_index_padded = 0;
    unsigned int padding_offset_y = size_x_ + 2 * padding_size;

    for (unsigned int row_index = 0; row_index < size_y_; ++row_index)
    {
      linear_index = row_index * size_x_;
      linear_index_padded = (row_index + padding_size) * padding_offset_y + padding_size;

      for(unsigned int column_index = 0; column_index < size_x_; ++column_index)
      {
        mask = padded_grid_mask[linear_index_padded];

        //if only one of mask_overflow_test bits is set we are not clearing the cell
        //so we had an overflow
        if (((mask & mask_overflow_test) ^ mask_overflow_test) == (unsigned int)1)
        {
          ROS_WARN("Removed overflow, was not tested before. Delete the warning + print if working correctly");
          printMask(mask);
          mask &= ~(mask_overflow_test); //remove overflow
        }

        //if only one of mask_underflow_test bits is set we are not clearing the cell
        //so we had an underflow
        if (((mask & mask_underflow_test) ^ mask_underflow_test) == ((unsigned int)1 << 31))
        {
          mask &= ~(mask_underflow_test); //remove underflow
        }

        ///TODO: Remove this after thorough testing
        if((mask >> 16) ^ (mask & mask_lower_bits))
        {
          ROS_ERROR("Error in voxel grid: Clearing ended up wrong. Seems like there is an bug in the ray tracing (with padding) algorithm!");
          linear_index++;
          linear_index_padded++;
          continue;
        }

        data_[linear_index] &= ~(mask);

//        printMask(padded_grid_mask[linear_index_padded]);

        linear_index++;
        linear_index_padded++;
      }
    }
  }

  void updateCostmap(unsigned char* costmap, bool* updatedColumns, unsigned int unknown_clear_threshold, unsigned int marked_clear_threshold,
                     unsigned char free_cost = 0, unsigned char unknown_cost = 255, unsigned int padding_size = 0)
  {
    unsigned int linear_index = 0;
    unsigned int linear_index_padded = 0;
    unsigned int padding_offset_y = size_x_ + (2 * padding_size);

    for (unsigned int row_index = 0; row_index < size_y_; ++row_index)
    {
      linear_index = row_index * size_x_;
      linear_index_padded = (row_index + padding_size) * padding_offset_y + padding_size;

      for(unsigned int column_index = 0; column_index < size_x_; ++column_index)
      {
        if(!updatedColumns[linear_index_padded])
        {
          linear_index++;
          linear_index_padded++;
          continue;
        }

        uint32_t column_occupation = data_[linear_index];

        unsigned int unknown_bits = uint16_t(column_occupation >> 16) ^ uint16_t(column_occupation);
        unsigned int marked_bits = column_occupation >> 16;

        if (bitsBelowThreshold(marked_bits, marked_clear_threshold))
        {
          if (bitsBelowThreshold(unknown_bits, unknown_clear_threshold))
          {
            costmap[linear_index] = free_cost;
          }
          else
          {
            costmap[linear_index] = unknown_cost;
          }
        }

        linear_index++;
        linear_index_padded++;
      }
    }
  }

  class Updater
  {
  public:
    Updater(uint32_t* data, bool* updated_columns) :
        data_(data), updated_columns_(updated_columns)
    {
    }

    inline void operator()(unsigned int offset, unsigned int z_mask)
    {
      data_[offset] |= z_mask;
      updated_columns_[offset] = true;
    }

  private:
    uint32_t* data_;
    bool* updated_columns_;
  };

  class UpdaterPaddedDominantX
  {
  public:
    UpdaterPaddedDominantX(uint32_t* padded_data, bool* updated_columns, unsigned int offset_y) :
      padded_data_(padded_data), updated_columns_(updated_columns), offset_y_(offset_y)
    {
    }

    inline void operator()(unsigned int offset, unsigned int z_mask)
    {
      padded_data_[offset] |= z_mask;
      updated_columns_[offset] = true;

      //padding z
      padded_data_[offset] |= z_mask << 1; //don't need to check for overflow because & ~(0) changes nothing, hopefully
      padded_data_[offset] |= z_mask >> 1; //don't need to check for underflow because & ~(0) changes nothing

      //padding y, data is padded so no under / overflow should happen
      padded_data_[offset+offset_y_] |= z_mask;
      updated_columns_[offset+offset_y_] = true;

      padded_data_[offset-offset_y_] |= z_mask;
      updated_columns_[offset-offset_y_] = true;
    }

  private:
    uint32_t* padded_data_;
    bool* updated_columns_;
    unsigned int offset_y_;
  };

  class UpdaterPaddedDominantY
  {
  public:
    UpdaterPaddedDominantY(uint32_t* padded_data, bool* updated_columns) :
      padded_data_(padded_data), updated_columns_(updated_columns)
    {
    }

    inline void operator()(unsigned int offset, unsigned int z_mask)
    {
      padded_data_[offset] |= z_mask;
      updated_columns_[offset] = true;

      //padding z
      padded_data_[offset] |= z_mask << 1; //don't need to check for overflow because & ~(0) changes nothing
      padded_data_[offset] |= z_mask >> 1; //don't need to check for underflow because & ~(0) changes nothing

      //padding x, data is padded so no under / overflow should happen
      padded_data_[offset+1] |= z_mask;
      updated_columns_[offset+1] = true;

      padded_data_[offset-1] |= z_mask;
      updated_columns_[offset-1] = true;
    }

  private:
    uint32_t* padded_data_;
    bool* updated_columns_;
  };

  class UpdaterPaddedDominantZ
  {
  public:
    UpdaterPaddedDominantZ(uint32_t* padded_data, bool* updated_columns, unsigned int offset_y) :
      padded_data_(padded_data), updated_columns_(updated_columns), offset_y_(offset_y)
    {
    }

    inline void operator()(unsigned int offset, unsigned int z_mask)
    {
      padded_data_[offset] &= z_mask;
      updated_columns_[offset] = true;

      //padding x, data is padded so no under / overflow should happen
      padded_data_[offset+1] &= z_mask;
      updated_columns_[offset+1] = true;

      padded_data_[offset-1] &= z_mask;
      updated_columns_[offset-1] = true;

      //padding y, data is padded so no under / overflow should happen
      padded_data_[offset+offset_y_] &= z_mask;
      updated_columns_[offset+offset_y_] = true;

      padded_data_[offset-offset_y_] &= z_mask;
      updated_columns_[offset-offset_y_] = true;
    }

  private:
    uint32_t* padded_data_;
    bool* updated_columns_;
    unsigned int offset_y_;
  };

  void printVoxelGrid();
  void printColumnGrid();
  unsigned int sizeX();
  unsigned int sizeY();
  unsigned int sizeZ();

  template <class ActionType>
  inline void raytraceLine(
    ActionType& at, double x0, double y0, double z0,
    double x1, double y1, double z1, unsigned int max_length = UINT_MAX)
  {
    int dx = int(x1) - int(x0);
    int dy = int(y1) - int(y0);
    int dz = int(z1) - int(z0);

    unsigned int abs_dx = abs(dx);
    unsigned int abs_dy = abs(dy);
    unsigned int abs_dz = abs(dz);

    int offset_dx = sign(dx);
    int offset_dy = sign(dy) * size_x_;
    int offset_dz = sign(dz);

    unsigned int z_mask = ((1 << 16) | 1) << (unsigned int)z0;
    unsigned int offset = (unsigned int)y0 * size_x_ + (unsigned int)x0;

    GridOffset grid_off(offset);
    ZOffset z_off(z_mask);

    //we need to chose how much to scale our dominant dimension, based on the maximum length of the line
    double dist = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) + (z0 - z1) * (z0 - z1));
    double scale = std::min(1.0,  max_length / dist);

    //is x dominant
    if (abs_dx >= max(abs_dy, abs_dz))
    {
      int error_y = abs_dx / 2;
      int error_z = abs_dx / 2;

      bresenham3D(at, grid_off, grid_off, z_off, abs_dx, abs_dy, abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz, offset, z_mask, (unsigned int)(scale * abs_dx));
      return;
    }

    //y is dominant
    if (abs_dy >= abs_dz)
    {
      int error_x = abs_dy / 2;
      int error_z = abs_dy / 2;

      bresenham3D(at, grid_off, grid_off, z_off, abs_dy, abs_dx, abs_dz, error_x, error_z, offset_dy, offset_dx, offset_dz, offset, z_mask, (unsigned int)(scale * abs_dy));
      return;
    }

    //otherwise, z is dominant
    int error_x = abs_dz / 2;
    int error_y = abs_dz / 2;

    bresenham3D(at, z_off, grid_off, grid_off, abs_dz, abs_dx, abs_dy, error_x, error_y, offset_dz, offset_dx, offset_dy, offset, z_mask, (unsigned int)(scale * abs_dz));
  }

private:
  //the real work is done here... 3D bresenham implementation
  template <class ActionType, class OffA, class OffB, class OffC>
  inline void bresenham3D(
    ActionType& at, OffA off_a, OffB off_b, OffC off_c,
    unsigned int abs_da, unsigned int abs_db, unsigned int abs_dc,
    int error_b, int error_c, int offset_a, int offset_b, int offset_c, unsigned int &offset,
    unsigned int &z_mask, unsigned int max_length = UINT_MAX)
  {
    unsigned int end = std::min(max_length, abs_da);
    for (unsigned int i = 0; i < end; ++i)
    {
      at(offset, z_mask);
      off_a(offset_a);
      error_b += abs_db;
      error_c += abs_dc;
      if ((unsigned int)error_b >= abs_da)
      {
        //adjusted not original Bresenham
        at(offset, z_mask); //set voxel on the same line
        off_b(offset_b); //go one line up
        off_a(-offset_a); //go one column back
        at(offset, z_mask); //set voxel
        off_a(offset_a); //go back to next column
        error_b -= abs_da;
      }
      if ((unsigned int)error_c >= abs_da)
      {
        //adjusted not original Bresenham
        at(offset, z_mask); //set voxel on the same line
        off_c(offset_c); //go one line up
        off_a(-offset_a); //go one column back
        at(offset, z_mask); //set voxel
        off_a(offset_a); //go back to next column
        error_c -= abs_da;
      }
    }
    at(offset, z_mask);
  }

  inline int sign(int i)
  {
    return i > 0 ? 1 : -1;
  }

  inline unsigned int max(unsigned int x, unsigned int y)
  {
    return x > y ? x : y;
  }

  unsigned int size_x_, size_y_, size_z_;
  uint32_t *data_;
  unsigned char *costmap_;

  //Aren't functors so much fun... used to recreate the Bresenham macro Eric wrote in the original version, but in "proper" c++
  class MarkVoxel
  {
  public:
    MarkVoxel(uint32_t* data): data_(data){}
    inline void operator()(unsigned int offset, unsigned int z_mask)
    {
      data_[offset] |= z_mask; //clear unknown and mark cell
    }
  private:
    uint32_t* data_;
  };

  class ClearVoxel
  {
  public:
    ClearVoxel(uint32_t* data): data_(data){}
    inline void operator()(unsigned int offset, unsigned int z_mask)
    {
      data_[offset] &= ~(z_mask); //clear unknown and clear cell
    }
  private:
    uint32_t* data_;
  };

  class ClearVoxelInMap
  {
  public:
    ClearVoxelInMap(uint32_t* data, unsigned char *costmap, unsigned int unknown_clear_threshold,
                    unsigned int marked_clear_threshold, unsigned char free_cost = 0, unsigned char unknown_cost = 255) :
        data_(data), costmap_(costmap), unknown_clear_threshold_(unknown_clear_threshold), marked_clear_threshold_(
            marked_clear_threshold), free_cost_(free_cost), unknown_cost_(unknown_cost)
    {
    }

    inline void operator()(unsigned int offset, unsigned int z_mask)
    {
      uint32_t* col = &data_[offset];
      *col &= ~(z_mask); //clear unknown and clear cell

      unsigned int unknown_bits = uint16_t(*col>>16) ^ uint16_t(*col);
      unsigned int marked_bits = *col>>16;

      //make sure the number of bits in each is below our thesholds
      if (bitsBelowThreshold(marked_bits, marked_clear_threshold_))
      {
        if (bitsBelowThreshold(unknown_bits, unknown_clear_threshold_))
        {
          costmap_[offset] = free_cost_;
        }
        else
        {
          costmap_[offset] = unknown_cost_;
        }
      }
    }
  private:
    inline bool bitsBelowThreshold(unsigned int n, unsigned int bit_threshold)
    {
      unsigned int bit_count;
      for (bit_count = 0; n;)
      {
        ++bit_count;
        if (bit_count > bit_threshold)
        {
          return false;
        }
        n &= n - 1; //clear the least significant bit set
      }
      return true;
    }

    uint32_t* data_;
    unsigned char *costmap_;
    unsigned int unknown_clear_threshold_, marked_clear_threshold_;
    unsigned char free_cost_, unknown_cost_;
  };

  class GridOffset
  {
  public:
    GridOffset(unsigned int &offset) : offset_(offset) {}
    inline void operator()(int offset_val)
    {
      offset_ += offset_val;
    }
  private:
    unsigned int &offset_;
  };

  class ZOffset
  {
  public:
    ZOffset(unsigned int &z_mask) : z_mask_(z_mask) {}
    inline void operator()(int offset_val)
    {
      offset_val > 0 ? z_mask_ <<= 1 : z_mask_ >>= 1;
    }
  private:
    unsigned int & z_mask_;
  };
};

}  // namespace voxel_grid

#endif  // VOXEL_GRID_VOXEL_GRID_H
