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
#include "graceful_controller_ros/orientation_tools.hpp"

namespace graceful_controller
{

// Helper function to get yaw if "from" were to point towards "to"
double getRelativeYaw(const geometry_msgs::PoseStamped& from,
                      const geometry_msgs::PoseStamped& to)
{
  double dx = to.pose.position.x - from.pose.position.x;
  double dy = to.pose.position.y - from.pose.position.y;
  return std::atan2(dy, dx);
}

void setYaw(geometry_msgs::PoseStamped& pose, double yaw)
{
  pose.pose.orientation.z = sin(yaw / 2.0);
  pose.pose.orientation.w = cos(yaw / 2.0);
}

std::vector<geometry_msgs::PoseStamped>
addOrientations(const std::vector<geometry_msgs::PoseStamped>& path)
{
  std::vector<geometry_msgs::PoseStamped> oriented_path;
  oriented_path.resize(path.size());
  if (path.empty())
  {
    // This really shouldn't happen
    return oriented_path;
  }

  // The last pose will already be oriented since it is our goal
  oriented_path.back() = path.back();

  // For each pose, point at the next one
  for (size_t i = 0; i < oriented_path.size() - 1; ++i)
  {
    oriented_path[i] = path[i];
    double yaw = getRelativeYaw(path[i], path[i + 1]);
    setYaw(oriented_path[i], yaw);
  }

  return oriented_path;
}

std::vector<geometry_msgs::PoseStamped>
applyOrientationFilter(const std::vector<geometry_msgs::PoseStamped>& path,
                       double yaw_tolerance,
                       double gap_tolerance)
{
  std::vector<geometry_msgs::PoseStamped> filtered_path;
  filtered_path.reserve(path.size());
  if (path.empty())
  {
    // This really shouldn't happen
    return filtered_path;
  }

  // Always keep the first pose
  filtered_path.push_back(path.front());

  // Possibly filter some intermediate poses
  for (size_t i = 1; i < path.size() - 1; ++i)
  {
    // Get the yaw angle if the previous pose were to be pointing at this pose
    // We need to recompute because we might have dropped poses
    double yaw_previous = getRelativeYaw(filtered_path.back(), path[i]);

    // Get the yaw angle of this pose pointing at next pose
    double yaw_this = tf2::getYaw(path[i].pose.orientation);

    // Get the yaw angle if previous pose were to be pointing at next pose, filtering this pose
    double yaw_without = getRelativeYaw(filtered_path.back(), path[i+1]);

    // Determine if this pose is far off a direct line between previous and next pose
    if (fabs(angles::shortest_angular_distance(yaw_previous, yaw_without)) < yaw_tolerance &&
        fabs(angles::shortest_angular_distance(yaw_this, yaw_without)) < yaw_tolerance)
    {
      // Update previous heading in case we dropped some poses
      setYaw(filtered_path.back(), yaw_previous);
      // Add this pose to the filtered plan
      filtered_path.push_back(path[i]);
    }
    else if (std::hypot(path[i].pose.position.x - filtered_path.back().pose.position.x,
                        path[i].pose.position.y - filtered_path.back().pose.position.y) >= gap_tolerance)
    {
      ROS_DEBUG_NAMED("orientation_filter", "Including pose %lu to meet max_separation_dist", i);
      // Update previous heading in case we dropped some poses
      setYaw(filtered_path.back(), yaw_previous);
      // Add this pose to the filtered plan
      filtered_path.push_back(path[i]);
    }
    else
    {
      // Sorry pose, the plan is better without you :(
      ROS_DEBUG_NAMED("orientation_filter", "Filtering pose %lu", i);
    }
  }

  // Reset heading of what will be penultimate pose, in case we dropped some poses
  setYaw(filtered_path.back(), getRelativeYaw(filtered_path.back(), path.back()));

  // Always add the last pose, since this is our goal
  filtered_path.push_back(path.back());

  ROS_DEBUG_NAMED("orientation_filter", "Filtered %lu points from plan", path.size() - filtered_path.size());
  return filtered_path;
}

}  // namespace graceful_controller
