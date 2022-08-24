/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021-2022, Michael Ferguson
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
 * Author: Eitan Marder-Eppstein, Michael Ferguson
 *********************************************************************/

#ifndef GRACEFUL_CONTROLLER_ROS_GRACEFUL_CONTROLLER_ROS_HPP
#define GRACEFUL_CONTROLLER_ROS_GRACEFUL_CONTROLLER_ROS_HPP

#include <mutex>

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>

#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <graceful_controller/graceful_controller.hpp>
#include <graceful_controller_ros/orientation_tools.hpp>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <graceful_controller_ros/GracefulControllerConfig.h>

#include "graceful_controller_ros/visualization.hpp"

namespace graceful_controller
{
class GracefulControllerROS : public nav_core::BaseLocalPlanner
{
public:
  GracefulControllerROS();
  virtual ~GracefulControllerROS();

  /**
   * @brief Constructs the local planner
   * @param name The name to give this instance of the local planner
   * @param tf A pointer to a transform buffer
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
  virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Callback for dynamic reconfigure server.
   */
  void reconfigureCallback(GracefulControllerConfig& config, uint32_t level);

  /**
   * @brief Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the
   * robot base
   * @return True if a valid velocity command was found, false otherwise
   */
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief Check if the goal pose has been achieved by the local planner
   * @return True if achieved, false otherwise
   */
  virtual bool isGoalReached();

  /**
   * @brief Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Rotate the robot towards a goal.
   * @param pose The pose should always be in base frame!
   * @param cmd_vel The returned command velocity
   * @returns The computed angular error.
   */
  double rotateTowards(const geometry_msgs::PoseStamped& pose, geometry_msgs::Twist& cmd_vel);

  /**
   * @brief Rotate the robot towards an angle.
   * @param yaw The angle to rotate.
   * @param cmd_vel The returned command velocity.
   */
  void rotateTowards(double yaw, geometry_msgs::Twist& cmd_vel);

private:
  void velocityCallback(const std_msgs::Float32::ConstPtr& max_vel_x);

  /**
   * @brief Simulate a path.
   * @param target_pose Pose to simulate towards.
   * @param cmd_vel The returned command to execute.
   * @returns True if the path is valid.
   */
  bool simulate(const geometry_msgs::PoseStamped& target_pose, geometry_msgs::Twist& cmd_vel);

  ros::Publisher global_plan_pub_, local_plan_pub_, target_pose_pub_;
  ros::Subscriber max_vel_sub_;

  bool initialized_;
  GracefulControllerPtr controller_;

  tf2_ros::Buffer* buffer_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  geometry_msgs::TransformStamped robot_to_costmap_transform_;
  base_local_planner::LocalPlannerUtil planner_util_;
  base_local_planner::OdometryHelperRos odom_helper_;

  // Parameters and dynamic reconfigure
  std::mutex config_mutex_;
  dynamic_reconfigure::Server<GracefulControllerConfig>* dsrv_;
  double max_vel_x_;
  double min_vel_x_;
  double max_vel_theta_;
  double min_in_place_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  double decel_lim_x_;
  double scaling_vel_x_;
  double scaling_factor_;
  double scaling_step_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double xy_vel_goal_tolerance_;
  double yaw_vel_goal_tolerance_;
  double min_lookahead_;
  double max_lookahead_;
  double resolution_;
  double acc_dt_;
  double yaw_filter_tolerance_;
  double yaw_gap_tolerance_;
  bool prefer_final_rotation_;
  bool compute_orientations_;
  bool use_orientation_filter_;

  // Goal tolerance
  bool latch_xy_goal_tolerance_;
  bool goal_tolerance_met_;

  // Controls initial rotation towards path
  double initial_rotate_tolerance_;
  bool has_new_path_;

  // Optional visualization of colliding and non-colliding points checked
  ros::Publisher collision_point_pub_;
  visualization_msgs::MarkerArray* collision_points_;

  geometry_msgs::PoseStamped robot_pose_;
};

}  // namespace graceful_controller

#endif  // GRACEFUL_CONTROLLER_ROS_GRACEFUL_CONTROLLER_ROS_HPP
