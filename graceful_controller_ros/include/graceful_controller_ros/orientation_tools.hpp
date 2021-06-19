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

#ifndef GRACEFUL_CONTROLLER_ROS_ORIENTATION_TOOLS_HPP
#define GRACEFUL_CONTROLLER_ROS_ORIENTATION_TOOLS_HPP

#include <vector>
#include <geometry_msgs/PoseStamped.h>

namespace graceful_controller
{

/**
 * @brief Add orientation to each pose in a path.
 * @param path The path to have orientations added.
 * @returns The oriented path.
 */
std::vector<geometry_msgs::PoseStamped>
addOrientations(const std::vector<geometry_msgs::PoseStamped>& path);

/**
 * @brief Filter a path for orientation noise.
 * @param path The path to be filtered.
 * @param yaw_tolerance Maximum deviation allowed before a pose is filtered.
 * @param gap_tolerance Maximum distance between poses in the filtered path.
 * @returns The filtered path.
 */
std::vector<geometry_msgs::PoseStamped>
applyOrientationFilter(const std::vector<geometry_msgs::PoseStamped>& path,
                       double yaw_tolerance,
                       double gap_tolerance);

}  // namespace graceful_controller

#endif  // GRACEFUL_CONTROLLER_ROS_ORIENTATION_TOOLS_HPP
