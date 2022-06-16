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

#include <nav_msgs/msg/path.hpp>
#include <nav2_core/controller.hpp>
#include <graceful_controller/graceful_controller.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "graceful_controller_ros/orientation_tools.hpp"
#include "graceful_controller_ros/visualization.hpp"

namespace graceful_controller
{
class GracefulControllerROS : public nav2_core::Controller
{
public:
  GracefulControllerROS();
  virtual ~GracefulControllerROS();

  /**
   * @brief Constructs the local planner
   * @param node WeakPtr to the Lifecycle node
   * @param name The name to give this instance of the local planner
   * @param tf A pointer to a transform buffer
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& node,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  /**
   * @brief Method to cleanup resources.
   */
  virtual void cleanup();

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate();

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate();

  /**
   * @brief Calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the current goal checker the task is utilizing
   * @return The best command for the robot to drive
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::Twist& cmd_vel,
    nav2_core::GoalChecker * goal_checker);

  /**
   * @brief Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   */
  virtual void setPlan(const nav_msgs::msg::Path & path);

  /**
   * @brief Rotate the robot towards a goal.
   * @param pose The pose should always be in base frame!
   * @param velocity Current robot velocity.
   * @param cmd_vel The returned command velocity.
   * @returns The computed angular error.
   */
  double rotateTowards(const geometry_msgs::msg::PoseStamped& pose,
                       const geometry_msgs::msg::Twist& velocity,
                       geometry_msgs::msg::TwistStamped& cmd_vel);

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  virtual void setSpeedLimit(const double& speed_limit, const bool& percentage);

private:
  /**
   * @brief Simulate a path.
   * @param target_pose Pose to simulate towards.
   * @param velocity Current robot velocity.
   * @param cmd_vel The returned command to execute.
   * @returns True if the path is valid.
   */
  bool simulate(
    const geometry_msgs::msg::PoseStamped& target_pose,
    const geometry_msgs::msg::Twist& velocity,
    geometry_msgs::msg::TwistStamped& cmd_vel);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> target_pose_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> collision_points_pub_;
  
  bool initialized_;
  GracefulControllerPtr controller_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  geometry_msgs::msg::TransformStamped robot_to_costmap_transform_;
  nav_msgs::msg::Path global_plan_;

  // Parameters
  std::mutex config_mutex_;
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
  visualization_msgs::msg::MarkerArray* collision_points_;

  geometry_msgs::msg::PoseStamped robot_pose_;
};

}  // namespace graceful_controller

#endif  // GRACEFUL_CONTROLLER_ROS_GRACEFUL_CONTROLLER_ROS_HPP
