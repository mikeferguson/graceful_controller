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

#include <cmath>
#include <mutex>

#include <angles/angles.h>
#include <nav_2d_utils/parameters.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_util/line_iterator.hpp>
#include <rclcpp/logging.hpp>
#include "graceful_controller_ros/graceful_controller_ros.hpp"

using rclcpp_lifecycle::LifecyclePublisher;
using nav2_util::declare_parameter_if_not_declared;

namespace graceful_controller
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("graceful_controller");

double sign(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

/**
 * @brief Collision check the robot pose
 * @param x The robot x coordinate in costmap.global frame
 * @param y The robot y coordinate in costmap.global frame
 * @param theta The robot rotation in costmap.global frame
 * @param viz Optional message for visualizing collisions
 * @param inflation Ratio to expand the footprint
 */
bool isColliding(double x, double y, double theta,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap,
                 visualization_msgs::msg::MarkerArray* viz, double inflation = 1.0)
{
  unsigned mx, my;
  if (!costmap->getCostmap()->worldToMap(x, y, mx, my))
  {
    RCLCPP_DEBUG(LOGGER, "Path is off costmap (%f,%f)", x, y);
    addPointMarker(x, y, true, viz);
    return true;
  }

  if (inflation < 1.0)
  {
    RCLCPP_WARN(LOGGER, "Inflation ratio cannot be less than 1.0");
    inflation = 1.0;
  }

  // Get footprint (centered around robot)
  std::vector<geometry_msgs::msg::Point> spec = costmap->getRobotFootprint();

  // Expand footprint by desired infation
  for (size_t i = 0; i < spec.size(); ++i)
  {
    spec[i].x *= inflation;
    spec[i].y *= inflation;
  }

  // Transform footprint to robot pose
  std::vector<geometry_msgs::msg::Point> footprint;
  nav2_costmap_2d::transformFootprint(x, y, theta, spec, footprint);

  // If our footprint is less than 4 corners, treat as circle
  if (footprint.size() < 4)
  {
    if (costmap->getCostmap()->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      RCLCPP_DEBUG(LOGGER, "Collision along path at (%f,%f)", x, y);
      addPointMarker(x, y, true, viz);
      return true;
    }
    // Done collison checking
    return false;
  }

  // Do a complete collision check of the footprint boundary
  for (size_t i = 0; i < footprint.size(); ++i)
  {
    unsigned x0, y0, x1, y1;
    if (!costmap->getCostmap()->worldToMap(footprint[i].x, footprint[i].y, x0, y0))
    {
      RCLCPP_DEBUG(LOGGER, "Footprint point %lu is off costmap", i);
      addPointMarker(footprint[i].x, footprint[i].y, true, viz);
      return true;
    }
    addPointMarker(footprint[i].x, footprint[i].y, false, viz);

    size_t next = (i + 1) % footprint.size();
    if (!costmap->getCostmap()->worldToMap(footprint[next].x, footprint[next].y, x1, y1))
    {
      RCLCPP_DEBUG(LOGGER, "Footprint point %lu is off costmap", next);
      addPointMarker(footprint[next].x, footprint[next].y, true, viz);
      return true;
    }
    addPointMarker(footprint[next].x, footprint[next].y, false, viz);

    for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance())
    {
      if (costmap->getCostmap()->getCost(line.getX(), line.getY()) >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        RCLCPP_DEBUG(LOGGER, "Collision along path at (%f,%f)", x, y);
        return true;
      }
    }
  }

  // Not colliding
  return false;
}

GracefulControllerROS::GracefulControllerROS() : initialized_(false), has_new_path_(false), collision_points_(NULL)
{
}

GracefulControllerROS::~GracefulControllerROS()
{
  if (collision_points_)
  {
    delete collision_points_;
  }
}

void GracefulControllerROS::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & weak_node,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  if (initialized_)
  {
    RCLCPP_ERROR(LOGGER, "This planner has already been initialized, doing nothing.");
    return;
  }

  // Save important things
  node_ = weak_node;
  buffer_ = tf;
  costmap_ros_ = costmap_ros;
  name_ = name;

  auto node = node_.lock();
  if (!node)
  {
    throw std::runtime_error{"Failed to lock node"};
  }

  clock_ = node->get_clock();

  // Setup parameters
  declare_parameter_if_not_declared(node, name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name_ + ".min_vel_x", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".min_in_place_vel_theta", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, name_ + ".acc_lim_x", rclcpp::ParameterValue(2.5));
  declare_parameter_if_not_declared(node, name_ + ".acc_lim_theta", rclcpp::ParameterValue(3.2));
  declare_parameter_if_not_declared(node, name_ + ".acc_dt", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, name_ + ".max_lookahead", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".min_lookahead", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, name_ + ".initial_rotate_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(node, name_ + ".prefer_final_rotation", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, name_ + ".compute_orientations", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(node, name_ + ".use_orientation_filter", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, name_ + ".yaw_filter_tolerance", rclcpp::ParameterValue(0.785));
  declare_parameter_if_not_declared(node, name_ + ".yaw_gap_tolerance", rclcpp::ParameterValue(0.25));
  declare_parameter_if_not_declared(node, name_ + ".latch_xy_goal_tolerance", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, name_ + ".publish_collision_points", rclcpp::ParameterValue(false));
  declare_parameter_if_not_declared(node, name_ + ".k1", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, name_ + ".k2", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(node, name_ + ".beta", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(node, name_ + ".lambda", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, name_ + ".scaling_vel_x", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, name_ + ".scaling_factor", rclcpp::ParameterValue(0.0));
  declare_parameter_if_not_declared(node, name_ + ".scaling_step", rclcpp::ParameterValue(0.1));

  node->get_parameter(name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(name_ + ".min_vel_x", min_vel_x_);
  node->get_parameter(name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(name_ + ".min_in_place_vel_theta", min_in_place_vel_theta_);
  node->get_parameter(name_ + ".acc_lim_x", acc_lim_x_);
  node->get_parameter(name_ + ".acc_lim_theta", acc_lim_theta_);
  node->get_parameter(name_ + ".acc_dt", acc_dt_);
  node->get_parameter(name_ + ".max_lookahead", max_lookahead_);
  node->get_parameter(name_ + ".min_lookahead", min_lookahead_);
  node->get_parameter(name_ + ".initial_rotate_tolerance", initial_rotate_tolerance_);
  node->get_parameter(name_ + ".prefer_final_rotation", prefer_final_rotation_);
  node->get_parameter(name_ + ".compute_orientations", compute_orientations_);
  node->get_parameter(name_ + ".use_orientation_filter", use_orientation_filter_);
  node->get_parameter(name_ + ".yaw_filter_tolerance", yaw_filter_tolerance_);
  node->get_parameter(name_ + ".yaw_gap_tolerance", yaw_gap_tolerance_);
  node->get_parameter(name_ + ".latch_xy_goal_tolerance", latch_xy_goal_tolerance_);
  node->get_parameter(name_ + ".scaling_vel_x_", scaling_vel_x_);
  node->get_parameter(name_ + ".scaling_factor", scaling_factor_);
  node->get_parameter(name_ + ".scaling_step", scaling_step_);
  resolution_ = costmap_ros_->getCostmap()->getResolution();
  double k1, k2, beta, lambda;
  node->get_parameter(name_ + ".k1", k1);
  node->get_parameter(name_ + ".k2", k2);
  node->get_parameter(name_ + ".beta", beta);
  node->get_parameter(name_ + ".lambda", lambda);

  // Publishers (same topics as DWA/TrajRollout)
  global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/global_plan", 1);
  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/local_plan", 1);
  target_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(name_ + "/target_pose", 1);

  bool publish_collision_points;
  node->get_parameter(name_ + ".publish_collision_points", publish_collision_points);
  if (publish_collision_points)
  {
    // Create publisher
    collision_points_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(name_ + "/collision_points", 1);

    // Create message to publish
    collision_points_ = new visualization_msgs::msg::MarkerArray();
  }

  controller_ = std::make_shared<GracefulController>(k1,
                                                     k2,
                                                     min_vel_x_,
                                                     max_vel_x_,
                                                     acc_lim_x_,
                                                     max_vel_theta_,
                                                     beta,
                                                     lambda);

  initialized_ = true;
}

void GracefulControllerROS::cleanup()
{
  global_plan_pub_.reset();
  local_plan_pub_.reset();
  target_pose_pub_.reset();
  collision_points_pub_.reset();
}

void GracefulControllerROS::activate()
{
  global_plan_pub_->on_activate();
  local_plan_pub_->on_activate();
  target_pose_pub_->on_activate();
  if (collision_points_)
  {
    collision_points_pub_->on_activate();
  }
  has_new_path_ = false;
}

void GracefulControllerROS::deactivate()
{
  global_plan_pub_->on_deactivate();
  local_plan_pub_->on_deactivate();
  target_pose_pub_->on_deactivate();
  if (collision_points_)
  {
    collision_points_pub_->on_deactivate();
  }
}

geometry_msgs::msg::TwistStamped GracefulControllerROS::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const geometry_msgs::msg::Twist& velocity,
  nav2_core::GoalChecker * goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;

  if (!initialized_)
  {
    RCLCPP_ERROR(LOGGER, "Controller is not initialized, call configure() before using this planner");
    return cmd_vel;
  }

  // Do this here to avoid segfault if not initialized
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  // Lock the mutex
  std::lock_guard<std::mutex> lock(config_mutex_);

  // Publish the global plan
  global_plan_pub_->publish(global_plan_);

  // Get transforms
  geometry_msgs::msg::TransformStamped plan_to_robot;
  try
  {
    plan_to_robot = buffer_->lookupTransform(costmap_ros_->getBaseFrameID(),
                                             global_plan_.header.frame_id,
                                             tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Could not transform to %s", costmap_ros_->getBaseFrameID().c_str());
    return cmd_vel;
  }

  try
  {
    robot_to_costmap_transform_ = buffer_->lookupTransform(costmap_ros_->getGlobalFrameID(),
                                                           costmap_ros_->getBaseFrameID(),
                                                           tf2::TimePointZero);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(LOGGER, "Could not transform to %s", costmap_ros_->getGlobalFrameID().c_str());
    return cmd_vel;
  }

  // Get the overall goal (in the robot frame)
  geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();
  tf2::doTransform(goal_pose, goal_pose, plan_to_robot);

  // Get goal tolerances
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  goal_checker->getTolerances(pose_tolerance, velocity_tolerance);

  // Compute distance to goal
  double dist_to_goal = std::hypot(goal_pose.pose.position.x, goal_pose.pose.position.y);

  // If we've reached the XY goal tolerance, just rotate
  if (dist_to_goal < pose_tolerance.position.x || goal_tolerance_met_)
  {
    // Reached goal, latch if desired
    goal_tolerance_met_ = latch_xy_goal_tolerance_;
    // Compute velocity required to rotate towards goal
    rotateTowards(goal_pose, velocity, cmd_vel);
    // Check for collisions between our current pose and goal
    double yaw_delta = tf2::getYaw(goal_pose.pose.orientation);
    size_t num_steps = fabs(yaw_delta) / 0.1;
    // Need to check at least the end pose
    num_steps = std::max(static_cast<size_t>(1), num_steps);
    for (size_t i = 1; i <= num_steps; ++i)
    {
      double step = static_cast<double>(i) / static_cast<double>(num_steps);
      double yaw = step * yaw_delta;
      if (isColliding(robot_pose.pose.position.x, robot_pose.pose.position.y, yaw, costmap_ros_, collision_points_))
      {
        RCLCPP_ERROR(LOGGER, "Unable to rotate in place due to collision");
        if (collision_points_ && !collision_points_->markers.empty())
        {
          collision_points_->markers[0].header.stamp = clock_->now();
          collision_points_pub_->publish(*collision_points_);
        }
        // Reset to zero velocity
        cmd_vel.twist = geometry_msgs::msg::Twist();
        return cmd_vel;
      }
    }
    // Safe to rotate, execute computed command
    return cmd_vel;
  }

  // Get controller max velocity based on current speed
  double max_vel_x = velocity.linear.x + (acc_lim_x_ * acc_dt_);
  max_vel_x = std::max(min_vel_x_, std::min(max_vel_x, max_vel_x_));

  // Work back from the end of plan to find valid target pose
  for (int i = global_plan_.poses.size() - 1; i >= 0; --i)
  {
    // Underlying control law needs a single target pose, which should:
    //  * Be as far away as possible from the robot (for smoothness)
    //  * But no further than the max_lookahed_ distance
    //  * Be feasible to reach in a collision free manner
    geometry_msgs::msg::PoseStamped target_pose;

    // Transform potential target pose into base_link
    tf2::doTransform(global_plan_.poses[i], target_pose, plan_to_robot);

    // Continue if target_pose is too far away from robot
    double dist_to_target = std::hypot(target_pose.pose.position.x, target_pose.pose.position.y);
    if (dist_to_target > max_lookahead_)
    {
      continue;
    }

    if (dist_to_goal < max_lookahead_)
    {
      if (prefer_final_rotation_)
      {
        // Avoid unstability and big sweeping turns at the end of paths by
        // ignoring final heading
        double yaw = std::atan2(target_pose.pose.position.y, target_pose.pose.position.x);
        target_pose.pose.orientation.z = sin(yaw / 2.0);
        target_pose.pose.orientation.w = cos(yaw / 2.0);
      }
    }
    else if (dist_to_target < min_lookahead_)
    {
      // Make sure target is far enough away to avoid instability
      break;
    }

    // Iteratively try to find a path, incrementally reducing the velocity
    double sim_velocity = max_vel_x;
    do
    {
      // Configure controller max velocity
      controller_->setVelocityLimits(min_vel_x_, sim_velocity, max_vel_theta_);
      // Actually simulate our path
      if (simulate(target_pose, velocity, cmd_vel))
      {
        // Have valid command
        return cmd_vel;
      }
      // Reduce velocity and try again for same target_pose
      sim_velocity -= scaling_step_;
    }
    while (sim_velocity >= scaling_vel_x_);
  }

  RCLCPP_ERROR(LOGGER, "No pose in path was reachable");
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 0.0;
  return cmd_vel;
}
 
bool GracefulControllerROS::simulate(
  const geometry_msgs::msg::PoseStamped& target_pose,
  const geometry_msgs::msg::Twist& velocity,
  geometry_msgs::msg::TwistStamped& cmd_vel)
{
  // Simulated path (for debugging/visualization)
  nav_msgs::msg::Path simulated_path;
  // Should we simulate rotation initially
  bool sim_initial_rotation_ = has_new_path_ && initial_rotate_tolerance_ > 0.0;
  // Clear any previous visualizations
  if (collision_points_)
  {
    collision_points_->markers.resize(0);
  }
  // Get control and path, iteratively
  while (true)
  {
    // The error between current robot pose and the target pose
    geometry_msgs::msg::PoseStamped error = target_pose;

    // Extract error_angle
    double error_angle = tf2::getYaw(error.pose.orientation);

    // Move origin to our current simulated pose
    if (!simulated_path.poses.empty())
    {
      double x = error.pose.position.x - simulated_path.poses.back().pose.position.x;
      double y = error.pose.position.y - simulated_path.poses.back().pose.position.y;

      double theta = -tf2::getYaw(simulated_path.poses.back().pose.orientation);
      error.pose.position.x = x * cos(theta) - y * sin(theta);
      error.pose.position.y = y * cos(theta) + x * sin(theta);

      error_angle += theta;
      error.pose.orientation.z = sin(error_angle / 2.0);
      error.pose.orientation.w = cos(error_angle / 2.0);
    }

    // Compute commands
    double vel_x, vel_th;
    if (sim_initial_rotation_)
    {
      geometry_msgs::msg::TwistStamped rotation;
      if (fabs(rotateTowards(error, velocity, rotation)) < initial_rotate_tolerance_)
      {
        if (simulated_path.poses.empty())
        {
          // Current robot pose satisifies initial rotate tolerance
          RCLCPP_INFO(LOGGER, "Done rotating towards path");
          has_new_path_ = false;
        }
        sim_initial_rotation_ = false;
      }
      vel_x = rotation.twist.linear.x;
      vel_th = rotation.twist.angular.z;
    }

    if (!sim_initial_rotation_)
    {
      if (!controller_->approach(error.pose.position.x, error.pose.position.y, error_angle,
                                 vel_x, vel_th))
      {
        RCLCPP_ERROR(LOGGER, "Unable to compute approach");
        return false;
      }
    }

    if (simulated_path.poses.empty())
    {
      // First iteration of simulation, store our commands to the robot
      cmd_vel.twist.linear.x = vel_x;
      cmd_vel.twist.angular.z = vel_th;
    }
    else if (std::hypot(error.pose.position.x, error.pose.position.y) < resolution_)
    {
      // We've simulated to the desired pose, can return this result
      local_plan_pub_->publish(simulated_path);
      target_pose_pub_->publish(target_pose);
      // Publish visualization if desired
      if (collision_points_ && !collision_points_->markers.empty())
      {
        collision_points_->markers[0].header.stamp = clock_->now();
        collision_points_pub_->publish(*collision_points_);
      }
      return true;
    }

    // Forward simulate command
    geometry_msgs::msg::PoseStamped next_pose;
    next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
    if (simulated_path.poses.empty())
    {
      // Initialize at origin
      next_pose.pose.orientation.w = 1.0;
    }
    else
    {
      // Start at last pose
      next_pose = simulated_path.poses.back();
    }

    // Generate next pose
    double dt = (vel_x > 0.0) ? resolution_ / vel_x : 0.1;
    double yaw = tf2::getYaw(next_pose.pose.orientation);
    next_pose.pose.position.x += dt * vel_x * cos(yaw);
    next_pose.pose.position.y += dt * vel_x * sin(yaw);
    yaw += dt * vel_th;
    next_pose.pose.orientation.z = sin(yaw / 2.0);
    next_pose.pose.orientation.w = cos(yaw / 2.0);
    simulated_path.poses.push_back(next_pose);

    // Compute footprint scaling
    double footprint_scaling = 1.0;
    if (vel_x > scaling_vel_x_)
    {
      // Scaling = (vel_x - scaling_vel_x) / (max_vel_x - scaling_vel_x)
      // NOTE: max_vel_x_ is possibly changing from ROS topic
      double ratio = max_vel_x_ - scaling_vel_x_;
      // Avoid divide by zero
      if (ratio > 0)
      {
        ratio = (vel_x - scaling_vel_x_) / ratio;
        footprint_scaling += ratio * scaling_factor_;
      }
    }

    // Check next pose for collision
    tf2::doTransform(next_pose, next_pose, robot_to_costmap_transform_);
    if (isColliding(next_pose.pose.position.x, next_pose.pose.position.y, tf2::getYaw(next_pose.pose.orientation),
                    costmap_ros_, collision_points_, footprint_scaling))
    {
      // Publish visualization if desired
      if (collision_points_ && !collision_points_->markers.empty())
      {
        collision_points_->markers[0].header.stamp = clock_->now();
        collision_points_pub_->publish(*collision_points_);
      }
      // Reason will be printed in function
      return false;
    }
  }

  // Really shouldn't hit this
  RCLCPP_ERROR(LOGGER, "Did not reach target_pose, but stopped simulating?");
  return false;
}

void GracefulControllerROS::setPlan(const nav_msgs::msg::Path & path)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(LOGGER, "Controller is not initialized, call initialize() before using this controller");
    return;
  }

  // We need orientations on our poses
  nav_msgs::msg::Path oriented_plan;
  if (compute_orientations_)
  {
    oriented_plan = addOrientations(path);
  }
  else
  {
    oriented_plan = path;
  }

  // Filter noisy orientations (if desired)
  nav_msgs::msg::Path filtered_plan;
  if (use_orientation_filter_)
  {
    filtered_plan = applyOrientationFilter(oriented_plan, yaw_filter_tolerance_, yaw_gap_tolerance_);
  }
  else
  {
    filtered_plan = oriented_plan;
  }

  // Store the plan for computeVelocityCommands
  global_plan_ = filtered_plan;
  has_new_path_ = true;
  goal_tolerance_met_ = false;
  RCLCPP_INFO(LOGGER, "Recieved a new path with %lu points in the %s frame", filtered_plan.poses.size(),
              global_plan_.header.frame_id.c_str());
}

double GracefulControllerROS::rotateTowards(
  const geometry_msgs::msg::PoseStamped& pose,
  const geometry_msgs::msg::Twist& velocity,
  geometry_msgs::msg::TwistStamped& cmd_vel)
{
  // Determine error
  double yaw = 0.0;
  if (std::hypot(pose.pose.position.x, pose.pose.position.y) > 0.5)
  {
    // Goal is far away, point towards goal
    yaw = std::atan2(pose.pose.position.y, pose.pose.position.x);
  }
  else
  {
    // Goal is nearby, align heading
    tf2::Quaternion orientation;
    yaw = tf2::getYaw(pose.pose.orientation);
  }

  RCLCPP_DEBUG(LOGGER, "Rotating towards goal, error = %f", yaw);

  // Determine max velocity based on current speed
  double max_vel_th = max_vel_theta_;
  if (acc_dt_ > 0.0)
  {
    double abs_vel = fabs(velocity.angular.z);
    double acc_limited = abs_vel + (acc_lim_theta_ * acc_dt_);
    max_vel_th = std::min(max_vel_th, acc_limited);
    max_vel_th = std::max(max_vel_th, min_in_place_vel_theta_);
  }

  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.angular.z = 2 * acc_lim_theta_ * fabs(yaw);
  cmd_vel.twist.angular.z = sign(yaw) * std::min(max_vel_th, std::max(min_in_place_vel_theta_, cmd_vel.twist.angular.z));

  // Return error
  return yaw;
}

void GracefulControllerROS::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  // Lock the mutex
  std::lock_guard<std::mutex> lock(config_mutex_);

  // TODO: handle percentage
  max_vel_x_ = std::max(static_cast<double>(speed_limit), min_vel_x_);
}

}  // namespace graceful_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(graceful_controller::GracefulControllerROS, nav2_core::Controller)
