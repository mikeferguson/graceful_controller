/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Michael Ferguson
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
* Author: Michael Ferguson
*********************************************************************/

#include <gtest/gtest.h>
#include <tf2/utils.h>  // getYaw
#include "graceful_controller_ros/orientation_tools.hpp"

using namespace graceful_controller;

TEST(OrientationToolsTests, test_bad_penultimate_pose)
{
  // Most global planners plan on a discrete grid,
  // but then add the start/end poses in continuous space.
  // This can lead to an odd orientations near the start/end of the path. 
  std::vector<geometry_msgs::PoseStamped> path;
  for (size_t i = 0; i < 10; ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = i * 0.05;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.push_back(pose);
  }

  // Set final pose to be off the grid and slightly BACK from last discrete pose
  path.back().pose.position.x = 0.378;
  path.back().pose.position.y = 0.02;
  // Set final pose to have a small rotation
  {
    double final_yaw = 0.15;
    path.back().pose.orientation.z = sin(final_yaw / 2.0);
    path.back().pose.orientation.w = cos(final_yaw / 2.0);
  }

  // Apply orientations
  path = addOrientations(path);

  // Last orientation should be unaltered
  EXPECT_EQ(0.0, path.back().pose.orientation.x);
  EXPECT_EQ(0.0, path.back().pose.orientation.y);
  EXPECT_FLOAT_EQ(0.15, tf2::getYaw(path.back().pose.orientation));

  // Early orientations should be forward
  for (size_t i = 0; i < 8; ++i)
  {
    EXPECT_EQ(0.0, path[i].pose.orientation.x);
    EXPECT_EQ(0.0, path[i].pose.orientation.y);
    EXPECT_FLOAT_EQ(0.0, tf2::getYaw(path[i].pose.orientation));
  }

  // Penultimate pose should be crazy
  EXPECT_EQ(0.0, path[8].pose.orientation.x);
  EXPECT_EQ(0.0, path[8].pose.orientation.y);
  EXPECT_FLOAT_EQ(2.4037776, tf2::getYaw(path[8].pose.orientation));

  // Now filter the penultimate pose away
  std::vector<geometry_msgs::PoseStamped> filtered_path;
  filtered_path = applyOrientationFilter(path, 0.785 /* 45 degrees */, 0.25);

  // Should have removed one pose (penultimate one)
  EXPECT_EQ(9, static_cast<int>(filtered_path.size()));

  // Last orientation should be unaltered
  EXPECT_EQ(0.0, filtered_path.back().pose.orientation.x);
  EXPECT_EQ(0.0, filtered_path.back().pose.orientation.y);
  EXPECT_FLOAT_EQ(0.15, tf2::getYaw(filtered_path.back().pose.orientation));

  // Early orientations should be forward
  for (size_t i = 0; i < 7; ++i)
  {
    EXPECT_EQ(0.0, filtered_path[i].pose.orientation.x);
    EXPECT_EQ(0.0, filtered_path[i].pose.orientation.y);
    EXPECT_FLOAT_EQ(0.0, tf2::getYaw(filtered_path[i].pose.orientation));
  }

  // Penultimate pose should be "better"
  EXPECT_EQ(0.0, filtered_path[7].pose.orientation.x);
  EXPECT_EQ(0.0, filtered_path[7].pose.orientation.y);
  EXPECT_FLOAT_EQ(0.62024951, tf2::getYaw(filtered_path[7].pose.orientation));
}

TEST(OrientationToolsTests, test_bad_initial_pose)
{
  // Most global planners plan on a discrete grid,
  // but then add the start/end poses in continuous space.
  // This can lead to an odd orientations near the start/end of the path. 
  std::vector<geometry_msgs::PoseStamped> path;
  for (size_t i = 0; i < 10; ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = i * 0.05;
    pose.pose.position.y = 0.0;
    pose.pose.orientation.w = 1.0;
    path.push_back(pose);
  }

  // Set initial pose to be off the grid, leading to bad initial orientation
  path.front().pose.position.x = 0.051;
  path.front().pose.position.y = 0.01;
  // Set final pose to have a small rotation
  {
    double final_yaw = -0.12;
    path.back().pose.orientation.z = sin(final_yaw / 2.0);
    path.back().pose.orientation.w = cos(final_yaw / 2.0);
  }

  // Apply orientations
  path = addOrientations(path);

  // Last orientation should be unaltered
  EXPECT_EQ(0.0, path.back().pose.orientation.x);
  EXPECT_EQ(0.0, path.back().pose.orientation.y);
  EXPECT_FLOAT_EQ(-0.12, tf2::getYaw(path.back().pose.orientation));

  // Initial pose orientation should be crazy
  EXPECT_EQ(0.0, path.front().pose.orientation.x);
  EXPECT_EQ(0.0, path.front().pose.orientation.y);
  EXPECT_FLOAT_EQ(-1.670465, tf2::getYaw(path.front().pose.orientation));

  // Other orientations should be forward
  for (size_t i = 1; i < 9; ++i)
  {
    EXPECT_EQ(0.0, path[i].pose.orientation.x);
    EXPECT_EQ(0.0, path[i].pose.orientation.y);
    EXPECT_FLOAT_EQ(0.0, tf2::getYaw(path[i].pose.orientation));
  }

  // Now filter the initial pose away
  std::vector<geometry_msgs::PoseStamped> filtered_path;
  filtered_path = applyOrientationFilter(path, 0.785 /* 45 degrees */, 0.25);

  // Should have removed one pose
  EXPECT_EQ(9, static_cast<int>(filtered_path.size()));

  // Initial orientation should be better
  EXPECT_EQ(0.0, filtered_path.front().pose.orientation.x);
  EXPECT_EQ(0.0, filtered_path.front().pose.orientation.y);
  EXPECT_FLOAT_EQ(-0.2013171, tf2::getYaw(filtered_path.front().pose.orientation));

  // Other orientations should be forward
  for (size_t i = 1; i < 8; ++i)
  {
    EXPECT_EQ(0.0, filtered_path[i].pose.orientation.x);
    EXPECT_EQ(0.0, filtered_path[i].pose.orientation.y);
    EXPECT_FLOAT_EQ(0.0, tf2::getYaw(filtered_path[i].pose.orientation));
  }

  // Last orientation should be unaltered
  EXPECT_EQ(0.0, filtered_path.back().pose.orientation.x);
  EXPECT_EQ(0.0, filtered_path.back().pose.orientation.y);
  EXPECT_FLOAT_EQ(-0.12, tf2::getYaw(filtered_path.back().pose.orientation));
}

TEST(OrientationToolsTests, test_zigzag)
{
  // This is an entirely unlikely path
  // Any global planner creating this path should be garbage collected
  std::vector<geometry_msgs::PoseStamped> path;
  for (size_t i = 0; i < 20; ++i)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = i * 0.05;
    if (i % 2 == 0)
    {
      pose.pose.position.y = 0.15;
    }
    else
    {
      pose.pose.position.y = 0.0;
    }
    pose.pose.orientation.w = 1.0;
    path.push_back(pose);
  }

  // Apply orientations
  path = addOrientations(path);

  // Now filter this awful path
  std::vector<geometry_msgs::PoseStamped> filtered_path;
  filtered_path = applyOrientationFilter(path, 0.785 /* 45 degrees */, 1.0);

  // Should have removed some poses
  EXPECT_EQ(6, static_cast<int>(filtered_path.size()));

  // Make sure the distance between remaining poses isn't too far
  for (size_t i = 1; i < filtered_path.size(); ++i)
  {
    double dx = filtered_path[i].pose.position.x - filtered_path[i-1].pose.position.x;
    double dy = filtered_path[i].pose.position.y - filtered_path[i-1].pose.position.y;
    EXPECT_TRUE(std::hypot(dx, dy) < 0.25);
  }
}

TEST(OrientationToolsTests, test_yaw_gap_tolerance)
{
  // This test verifies that poses do not exceed the yaw_gap_tolerance
  std::vector<geometry_msgs::PoseStamped> path;

  // Initial pose is at origin
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;
  path.push_back(pose);

  // Next pose is back just a bit
  pose.pose.position.x = -0.05;
  pose.pose.position.y = -0.01;
  path.push_back(pose);

  // Add remaining poses
  for (size_t i = 0; i < 8; ++i)
  {
    pose.pose.position.x = -0.1;
    pose.pose.position.y = -0.01 + i * 0.05;
    path.push_back(pose);
  }

  // Update final pose to have a rotation
  {
    double final_yaw = 1.0;
    path.back().pose.orientation.z = sin(final_yaw / 2.0);
    path.back().pose.orientation.w = cos(final_yaw / 2.0);
  }

  // Apply orientations
  path = addOrientations(path);

  // Last orientation should be unaltered
  EXPECT_EQ(0.0, path.back().pose.orientation.x);
  EXPECT_EQ(0.0, path.back().pose.orientation.y);
  EXPECT_FLOAT_EQ(1.0, tf2::getYaw(path.back().pose.orientation));

  // Initial pose orientation should be way back
  EXPECT_EQ(0.0, path.front().pose.orientation.x);
  EXPECT_EQ(0.0, path.front().pose.orientation.y);
  EXPECT_FLOAT_EQ(-2.9441972, tf2::getYaw(path.front().pose.orientation));

  // Second pose orientation should be straight back
  EXPECT_EQ(0.0, path[1].pose.orientation.x);
  EXPECT_EQ(0.0, path[1].pose.orientation.y);
  EXPECT_FLOAT_EQ(3.1415927, tf2::getYaw(path[1].pose.orientation));

  // Other orientations should be up
  for (size_t i = 2; i < 9; ++i)
  {
    EXPECT_EQ(0.0, path[i].pose.orientation.x);
    EXPECT_EQ(0.0, path[i].pose.orientation.y);
    EXPECT_FLOAT_EQ(1.5707964, tf2::getYaw(path[i].pose.orientation));
  }

  // Now filter (without a max separation distance)
  std::vector<geometry_msgs::PoseStamped> filtered_path;
  filtered_path = applyOrientationFilter(path, 0.1, 10.0);
  // Removes lots of poses
  EXPECT_EQ(3, static_cast<int>(filtered_path.size()));

  // Now filter with decent max separation distance
  filtered_path = applyOrientationFilter(path, 0.1, 0.15);
  // Removes not so many poses
  EXPECT_EQ(7, static_cast<int>(filtered_path.size()));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
