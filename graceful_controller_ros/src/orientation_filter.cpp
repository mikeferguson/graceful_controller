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

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf2/utils.h>  // getYaw
#include "graceful_controller_ros/orientation_filter.hpp"

namespace graceful_controller
{

std::vector<geometry_msgs::PoseStamped>
apply_orientation_filter(std::vector<geometry_msgs::PoseStamped>& path,
                         double yaw_tolerance)
{
	std::vector<geometry_msgs::PoseStamped> filtered_path;
  filtered_path.reserve(path.size());
  filtered_path.push_back(path.front());
  for (size_t i = 1; i < path.size() - 1; ++i)
  {
    // Compare to before and after
    if (angles::shortest_angular_distance(tf2::getYaw(filtered_path.back().pose.orientation),
                                          tf2::getYaw(path[i].pose.orientation)) < yaw_tolerance)
    {
      filtered_path.push_back(path[i]);
    }
    else
    {
      ROS_DEBUG_NAMED("orientation_filter", "Filtering pose %lu", i);
    }
  }
  filtered_path.push_back(path.back());
  ROS_DEBUG_NAMED("orientation_filter", "Filtered %lu points from plan", path.size() - filtered_path.size());
	return filtered_path;
}

}  // namespace graceful_controller
