/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Michael Ferguson
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
 *********************************************************************/

#include "graceful_controller_ros/visualization.hpp"

void addPointMarker(double x, double y, bool colliding, visualization_msgs::MarkerArray* msg)
{
  if (!msg)
  {
    // Nothing to do, just return
    return;
  }

  if (msg->markers.empty())
  {
    // Add the marker
    msg->markers.resize(1);
    msg->markers[0].type = msg->markers[0].POINTS;
    msg->markers[0].header.frame_id = "odom";
    msg->markers[0].header.stamp = ros::Time::now();
    msg->markers[0].pose.orientation.w = 1.0;
    msg->markers[0].scale.x = 0.02;
    msg->markers[0].scale.y = 0.02;
    msg->markers[0].scale.z = 0.02;
  }

  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = 0.0;
  msg->markers[0].points.push_back(point);

  std_msgs::ColorRGBA color;
  if (colliding)
  {
    // Colliding points are RED
    color.r = 1;
    color.g = 0;
    color.b = 0;
  }
  else
  {
    // Non-colliding points are GREEN
    color.r = 0;
    color.g = 1;
    color.b = 0;
  }
  // All points are solid colored
  color.a = 1;
  msg->markers[0].colors.push_back(color);
}
