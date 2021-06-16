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

  // Apply orientations
  path = add_orientations(path);

  // Last orientation should be unaltered
  EXPECT_EQ(0.0, path.back().pose.orientation.x);
  EXPECT_EQ(0.0, path.back().pose.orientation.y);
  EXPECT_EQ(0.0, path.back().pose.orientation.z);
  EXPECT_EQ(1.0, path.back().pose.orientation.w);

  // Early orientations should be forward
  for (size_t i = 0; i < 8; ++i)
  {
    EXPECT_EQ(0.0, path[i].pose.orientation.x);
    EXPECT_EQ(0.0, path[i].pose.orientation.y);
    EXPECT_EQ(0.0, path[i].pose.orientation.z);
    EXPECT_EQ(1.0, path[i].pose.orientation.w);
  }

  // Penultimate pose should be crazy
  EXPECT_EQ(0.0, path[8].pose.orientation.x);
  EXPECT_EQ(0.0, path[8].pose.orientation.y);
  EXPECT_FLOAT_EQ(0.93272200, path[8].pose.orientation.z);
  EXPECT_FLOAT_EQ(0.36059669, path[8].pose.orientation.w);

  // Now filter the penultimate pose away
  std::vector<geometry_msgs::PoseStamped> filtered_path;
  filtered_path = apply_orientation_filter(path, 0.785 /* 45 degrees */);

  // Should have removed one pose
  EXPECT_EQ(9, static_cast<int>(filtered_path.size()));

  // Last orientation should be unaltered
  EXPECT_EQ(0.0, path.back().pose.orientation.x);
  EXPECT_EQ(0.0, path.back().pose.orientation.y);
  EXPECT_EQ(0.0, path.back().pose.orientation.z);
  EXPECT_EQ(1.0, path.back().pose.orientation.w);

  // Early orientations should be forward
  for (size_t i = 0; i < 8; ++i)
  {
    EXPECT_EQ(0.0, path[i].pose.orientation.x);
    EXPECT_EQ(0.0, path[i].pose.orientation.y);
    EXPECT_EQ(0.0, path[i].pose.orientation.z);
    EXPECT_EQ(1.0, path[i].pose.orientation.w);
  }
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

  // Apply orientations
  path = add_orientations(path);

  // Last orientation should be unaltered
  EXPECT_EQ(0.0, path.back().pose.orientation.x);
  EXPECT_EQ(0.0, path.back().pose.orientation.y);
  EXPECT_EQ(0.0, path.back().pose.orientation.z);
  EXPECT_EQ(1.0, path.back().pose.orientation.w);

  // Initial pose orientation should be crazy
  EXPECT_EQ(0.0, path.front().pose.orientation.x);
  EXPECT_EQ(0.0, path.front().pose.orientation.y);
  EXPECT_FLOAT_EQ(-0.74145252, path.front().pose.orientation.z);
  EXPECT_FLOAT_EQ(0.67100531, path.front().pose.orientation.w);

  // Other orientations should be forward
  for (size_t i = 1; i < 10; ++i)
  {
    EXPECT_EQ(0.0, path[i].pose.orientation.x);
    EXPECT_EQ(0.0, path[i].pose.orientation.y);
    EXPECT_EQ(0.0, path[i].pose.orientation.z);
    EXPECT_EQ(1.0, path[i].pose.orientation.w);
  }

  // Now filter the initial pose away
  std::vector<geometry_msgs::PoseStamped> filtered_path;
  filtered_path = apply_orientation_filter(path, 0.785 /* 45 degrees */);

  // Should have removed one pose
  // TODO: this fails with size 2 - we've removed all the poses except first and last
  EXPECT_EQ(9, static_cast<int>(filtered_path.size()));

  // Other orientations should be forward
  for (size_t i = 1; i < 10; ++i)
  {
    EXPECT_EQ(0.0, path[i].pose.orientation.x);
    EXPECT_EQ(0.0, path[i].pose.orientation.y);
    EXPECT_EQ(0.0, path[i].pose.orientation.z);
    EXPECT_EQ(1.0, path[i].pose.orientation.w);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
