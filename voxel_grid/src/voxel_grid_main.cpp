/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
#include <ros/console.h>
#include <utility>
#include <list>

inline int sign(int i)
{
  return i >= 0 ? 1 : -1;
}

inline unsigned int max(unsigned int x, unsigned int y)
{
  return x > y ? x : y;
}

std::list<std::pair<double, double> > bresen(int off_a, int off_b, int off_c, unsigned int abs_da, unsigned int abs_db,
                                             unsigned int abs_dc, int error_b, int error_c, int offset_a, int offset_b,
                                             int offset_c, unsigned int length)
{
  std::list<std::pair<double, double> > marked_cells;

//  ROS_ERROR("Abs: %i %i %i", (int)abs_da, (int)abs_db, (int)abs_dc);

//  ROS_ERROR("Error: %i", error_c);

  for (unsigned int i = 0; i < length; ++i)
  {
//    ROS_WARN("%i %i", off_a, off_c);
    marked_cells.push_back(std::make_pair(off_a, off_c));
    off_a += offset_a;

    if (error_b >= 0)
    {
      //adjusted not original Bresenham
//      ROS_INFO("b %i %i", off_a, off_c);
//      marked_cells.push_back(std::make_pair(off_a, off_c));

      off_b += offset_b; //go one line up
//      off_a -= offset_a; //go one column back

//      ROS_INFO("b %i %i", off_a, off_c);
//      marked_cells.push_back(std::make_pair(off_a, off_c));
//
//      off_a += offset_a; //go back to next column

      error_b -= abs_da;
    }

    if (error_c >= 0)
    {
      //adjusted not original Bresenham
//      ROS_INFO("%i %i", off_a, off_c);
      marked_cells.push_back(std::make_pair(off_a, off_c));

      off_c += offset_c; //go one line up
      off_a -= offset_a; //go one column back
//      ROS_INFO("%i %i", off_a, off_c);
      marked_cells.push_back(std::make_pair(off_a, off_c));

      off_a += offset_a; //go back to next column

      error_c -= abs_da;
    }

    error_b += abs_db;
    error_c += abs_dc;

//    ROS_ERROR("Error: %i", error_c);
  }
//  ROS_WARN("%i %i", off_a, off_c);
//  marked_cells.push_back(std::make_pair(off_a, off_c));

  return marked_cells;
}

std::list<std::pair<double, double> > test(double x0, double x1, double z0, double z1, double spacing)
{
  double dx = (x1 - x0);
  double dz = (z1 - z0);

  double m = dz / dx;
  double c = z0 - m * x0;

  std::list<std::pair<double, double> > marked_cells;

  double step = x0;
  double last_x = floor(x0);
  double last_y = floor(z0);
  double y = 0;

//  ROS_WARN("%i %i", (int )last_x, (int )last_y);
  marked_cells.push_back(std::make_pair(last_x, last_y));

  while (step <= x1)
  {
    y = m * step + c;

    if (floor(y) > last_y)
    {
      last_y = floor(y);
//      ROS_WARN("%i %i", (int )last_x, (int )last_y);
      marked_cells.push_back(std::make_pair(last_x, last_y));
    }

    if (floor(step) > last_x)
    {
      last_x = floor(step);
//      ROS_WARN("%i %i", (int )last_x, (int )last_y);
      marked_cells.push_back(std::make_pair(last_x, last_y));
    }

    step += 1 / spacing;
  }

  y = m * step + c;

  if (floor(y) > last_y)
  {
    last_y = floor(y);
//      ROS_WARN("%i %i", (int )last_x, (int )last_y);
    marked_cells.push_back(std::make_pair(last_x, last_y));
  }

  if (floor(step) > last_x)
  {
    last_x = floor(step);
//      ROS_WARN("%i %i", (int )last_x, (int )last_y);
    marked_cells.push_back(std::make_pair(last_x, last_y));
  }

  return marked_cells;
}

void vis(std::list<std::pair<double, double> > marked_cells, int max_x, int max_y)
{
  int y = 0;
  int x = 0;

  max_x -= 1;
  max_y -= 1;

  printf("\n");
  for (int k = 0; k <= max_x; ++k)
  {
    printf("--");
  }
  printf("\n");

  int row = 0;

  printf("%i ", row);
  ++row;

  for (std::list<std::pair<double, double> >::iterator it = marked_cells.begin(); it != marked_cells.end(); it++)
  {
    while ((int)it->second != y)
    {

      while (x <= max_x)
      {
        printf("| ");
        ++x;
      }

      x = 0;

      printf("\n");
      printf("%i ", row);
//      for (int k = 0; k <= max_x; ++k)
//      {
//        printf("--");x
//      }
//      printf("\n");

      ++y;
      ++row;
    }

    while ((int)it->first != x)
    {
      printf("| ");
      ++x;
    }

    printf("|x");
    ++x;
  }

  while (x <= max_x)
  {
    printf("| ");
    ++x;
  }

  x = 0;

  while (y != max_y)
  {
    printf("\n");
//    for (int k = 0; k <= max_x; ++k)
//    {
//      printf("--");
//    }
//    printf("\n");

    printf("%i ", row);
    while (x <= max_x)
    {
      printf("| ");
      ++x;
    }

    x = 0;

    ++y;
    ++row;
  }

  printf("\n");
//  for (int k = 0; k <= max_x; ++k)
//  {
//    printf("--");
//  }
//  printf("\n");
}

std::list<std::pair<double, double> > testBresen(double x0, double x1, double z0, double z1)
{
  double y0 = x0;
  double y1 = x1;

  double scaling_amount = pow(2, 16); //more is more accurate but lower max value (as we have limit of MAX_INT)

  double dx = (x1 - x0);
  double dy = (y1 - y0);
  double dz = (z1 - z0);

  double abs_dx = fabs(scaling_amount * dx);
  double abs_dy = fabs(scaling_amount * dy);
  double abs_dz = fabs(scaling_amount * dz);

  int offset_dx = sign(dx);
  int offset_dy = sign(dy);
  int offset_dz = sign(dz);

  unsigned int offset_x = (unsigned int)x0;
  unsigned int offset_y = (unsigned int)y0;
  unsigned int offset_z = (unsigned int)z0;

  double temp = 0;
  double x_lost_rounding = dx > 0 ? 1.0 - fabs(modf(x0, &temp)) : fabs(modf(x0, &temp));
  double y_lost_rounding = dy > 0 ? 1.0 - fabs(modf(y0, &temp)) : fabs(modf(y0, &temp));
  double z_lost_rounding = dz > 0 ? 1.0 - fabs(modf(z0, &temp)) : fabs(modf(z0, &temp));

//  ROS_INFO("abs %f, modf %f", abs(modf(x0, &temp)), modf(x0, &temp));
//  ROS_INFO("z_lost_rounding %f, x_lost_rounding %f", z_lost_rounding, x_lost_rounding);

  int error_y = (int)(abs_dy * x_lost_rounding - abs_dx * y_lost_rounding);
  int error_z = (int)(abs_dz * x_lost_rounding - abs_dx * z_lost_rounding);

  return bresen(offset_x, offset_y, offset_z, abs_dx, abs_dy, abs_dz, error_y, error_z, offset_dx, offset_dy, offset_dz,
                abs(int(y0) - int(y1)));
}

bool check(std::list<std::pair<double, double> > list1, std::list<std::pair<double, double> > list2)
{
  std::list<std::pair<double, double> >::iterator it_list2;

  double x = 0;
  double y = 0;
  bool found = false;

  for (std::list<std::pair<double, double> >::iterator it = list1.begin(); it != list1.end(); it++)
  {
    x = it->first;
    y = it->second;
    found = false;

    for (std::list<std::pair<double, double> >::iterator it2 = list2.begin(); it2 != list2.end(); it2++)
    {
      if (it2->first == x)
      {
        if (it2->second == y)
        {
          found = true;
          break;
        }
      }
    }

    if (!found)
    {
      ROS_ERROR("%f, %f not found", x, y);
      return false;
    }
  }

  return true;
}

int main(int argc, char *argv[])
{
  double max_x = 10;
  double max_y = 10;

  srand(time(NULL));

  int failures = 0;
  int successes = 0;
  int count = 0;

  for (int i = 0; i < 1000; ++i)
  {
    double x0 = (rand() % 100) / 10.0 + 0.01;// 0.75;
    double x1 = (rand() % 100) / 10.0 + 0.01;//4.3;
    double z0 = (rand() % 100) / max_y + 0.01;//0.25;
    double z1 = (rand() % 100) / max_y + 0.01;//2.8;

//     x0 = 39.010000; x1 = 83.210000; z0 = 6.810000; z1 = 7.010000;


    double dx = fabs(x0 - x1);
    double dz = fabs(z0 - z1);

    if(x0 >= x1 || z0 >= z1 || fabs(x0 - x1) < fabs(z0 - z1) || (dz / dx) == 1)
      continue;

    x0 = 5.510000; x1 = 8.310000; z0 = 7.510000; z1 = 8.710000;

    std::list<std::pair<double, double> > bresenResult = testBresen(x0, x1, z0, z1);
    ROS_INFO("--------------------------");
    std::list<std::pair<double, double> > testResult = test(x0, x1, z0, z1, 20000);

   ROS_INFO("x0 = %f; x1 = %f; z0 = %f; z1 = %f;", x0, x1, z0, z1);

   vis(bresenResult, max_x, max_y);
   vis(testResult, max_x, max_y);

   std::getchar();

    count++;

//    if (check(testResult, bresenResult))//, testResult))
//    {
//      ROS_INFO("x0 = %f; x1 = %f; z0 = %f; z1 = %f;", x0, x1, z0, z1);
//      ROS_WARN("Success");
//      successes++;
//    }
//    else
//    {
//      ROS_INFO("x0 = %f; x1 = %f; z0 = %f; z1 = %f;", x0, x1, z0, z1);
//
//      double m = dz / dx;
//      double c = z0 - m * x0;
//
//      ROS_INFO("y = %f * x + %f", m, c);
//
////      std::getchar();
////
//      vis(bresenResult, max_x, max_y);
//      vis(testResult, max_x, max_y);
//
//      ROS_ERROR("FAILURE");
//
//      failures++;
//
//      std::getchar();
//    }

    ROS_ERROR("##########################################");
  }

  ROS_INFO("Failure rate: %f", (double)failures / (double)count);
}
