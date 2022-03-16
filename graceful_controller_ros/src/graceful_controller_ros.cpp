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

#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>

#include <angles/angles.h>
#include <base_local_planner/line_iterator.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <costmap_2d/footprint.h>
#include <graceful_controller/graceful_controller.hpp>
#include <graceful_controller_ros/orientation_tools.hpp>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <graceful_controller_ros/GracefulControllerConfig.h>

namespace graceful_controller
{

double sign(double x)
{
  return x < 0.0 ? -1.0 : 1.0;
}

/**
 * @brief Collision check the robot pose
 * @param x The robot x coordinate in costmap.global frame
 * @param y The robot y coordinate in costmap.global frame
 * @param theta The robot rotation in costmap.global frame
 */
bool isColliding(double x, double y, double theta,
                 costmap_2d::Costmap2DROS* costmap)
{
  unsigned mx, my;
  if (!costmap->getCostmap()->worldToMap(x, y, mx, my))
  {
    ROS_DEBUG("Path is off costmap (%f,%f)", x, y);
    return true;
  }

  std::vector<geometry_msgs::Point> spec = costmap->getRobotFootprint();
  std::vector<geometry_msgs::Point> footprint;
  costmap_2d::transformFootprint(x, y, theta, spec, footprint);

  // If our footprint is less than 4 corners, treat as circle
  if (footprint.size() < 4)
  {
    if (costmap->getCostmap()->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      ROS_DEBUG("Collision along path at (%f,%f)", x, y);
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
      ROS_DEBUG("Footprint point %lu is off costmap", i);
      return true;
    }

    size_t next = (i + 1) % footprint.size();
    if (!costmap->getCostmap()->worldToMap(footprint[next].x, footprint[next].y, x1, y1))
    {
      ROS_DEBUG("Footprint point %lu is off costmap", next);
      return true;
    }

    for (base_local_planner::LineIterator line(x0,y0,x1,y1); line.isValid(); line.advance())
    {
      if (costmap->getCostmap()->getCost(line.getX(), line.getY()) >= costmap_2d::LETHAL_OBSTACLE)
      {
        ROS_DEBUG("Collision along path at (%f,%f)", x, y);
        return true;
      }
    }
  }

  // Not colliding
  return false;
}

class GracefulControllerROS : public nav_core::BaseLocalPlanner
{
public:
  GracefulControllerROS() :
    initialized_(false),
    has_new_path_(false)
  {
  }

  /**
   * @brief Constructs the local planner
   * @param name The name to give this instance of the local planner
   * @param tf A pointer to a transform buffer
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
  virtual void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (!initialized_)
    {
      // Publishers (same topics as DWA/TrajRollout)
      ros::NodeHandle private_nh("~/" + name);
      global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      target_pose_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("target_pose", 1);

      buffer_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      std::string odom_topic;
      if (private_nh.getParam("odom_topic", odom_topic))
      {
        odom_helper_.setOdomTopic(odom_topic);
        private_nh.param("acc_dt", acc_dt_, 0.25);
      }

      bool use_vel_topic = false;
      private_nh.getParam("use_vel_topic", use_vel_topic);
      if (use_vel_topic)
      {
        ros::NodeHandle nh;
        max_vel_sub_ = nh.subscribe<std_msgs::Float32>("max_vel_x", 1,
                         boost::bind(&GracefulControllerROS::velocityCallback, this, _1));
      }

      // Dynamic reconfigure is really only intended for tuning controller!
      dsrv_ = new dynamic_reconfigure::Server<GracefulControllerConfig>(private_nh);
      dynamic_reconfigure::Server<GracefulControllerConfig>::CallbackType cb =
        boost::bind(&GracefulControllerROS::reconfigureCallback, this, _1, _2);
      dsrv_->setCallback(cb);

      initialized_ = true;
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  /**
   * @brief Callback for dynamic reconfigure server.
   */
  void reconfigureCallback(GracefulControllerConfig &config, uint32_t level)
  {
    // Lock the mutex
    std::lock_guard<std::mutex> lock(config_mutex_);

    // Need to configure the planner utils
    base_local_planner::LocalPlannerLimits limits;
    limits.max_vel_trans = 0.0;
    limits.min_vel_trans = 0.0;
    limits.max_vel_x = config.max_vel_x;
    limits.min_vel_x = config.min_vel_x;
    limits.max_vel_y = 0.0;
    limits.min_vel_y = 0.0;
    limits.max_vel_theta = config.max_vel_theta;
    limits.acc_lim_x = config.acc_lim_x;
    limits.acc_lim_y = 0.0;
    limits.acc_lim_theta = config.acc_lim_theta;
    limits.acc_lim_trans = 0.0;
    limits.xy_goal_tolerance = config.xy_goal_tolerance;
    limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
    limits.prune_plan = false;
    limits.trans_stopped_vel = 0.0;
    limits.theta_stopped_vel = 0.0;
    planner_util_.reconfigureCB(limits, false);

    max_vel_x_ = config.max_vel_x;
    min_vel_x_ = config.min_vel_x;
    max_vel_theta_ = config.max_vel_theta;
    min_in_place_vel_theta_ = config.min_in_place_vel_theta;
    acc_lim_x_ = config.acc_lim_x;
    acc_lim_theta_ = config.acc_lim_theta;
    xy_goal_tolerance_ = config.xy_goal_tolerance;
    yaw_goal_tolerance_ = config.yaw_goal_tolerance;
    min_lookahead_ = config.min_lookahead;
    max_lookahead_ = config.max_lookahead;
    initial_rotate_tolerance_ = config.initial_rotate_tolerance;
    prefer_final_rotation_ = config.prefer_final_rotation;
    compute_orientations_ = config.compute_orientations;
    use_orientation_filter_ = config.use_orientation_filter;
    yaw_filter_tolerance_ = config.yaw_filter_tolerance;
    yaw_gap_tolerance_ = config.yaw_goal_tolerance;
    latch_xy_goal_tolerance_ = config.latch_xy_goal_tolerance;
    resolution_ = planner_util_.getCostmap()->getResolution();

    controller_ = std::make_shared<GracefulController>(config.k1,
                                                       config.k2,
                                                       config.min_vel_x,
                                                       config.max_vel_x,
                                                       config.acc_lim_x,
                                                       config.max_vel_theta,
                                                       config.beta,
                                                       config.lambda);
  }

  /**
   * @brief Given the current position, orientation, and velocity of the robot, compute
   *        velocity commands to send to the base
   * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   * @return True if a valid velocity command was found, false otherwise
   */
  virtual bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner is not initialized, call initialize() before using this planner");
      return false;
    }

    // Lock the mutex
    std::lock_guard<std::mutex> lock(config_mutex_);

    if (!costmap_ros_->getRobotPose(robot_pose_))
    {
      ROS_ERROR("Could not get the robot pose");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if (!planner_util_.getLocalPlan(robot_pose_, transformed_plan))
    {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    base_local_planner::publishPlan(transformed_plan, global_plan_pub_);

    if (transformed_plan.empty())
    {
      ROS_WARN("Received an empty transform plan");
      return false;
    }

    // Get transforms
    geometry_msgs::TransformStamped odom_to_base, base_to_odom;
    try
    {
      odom_to_base = buffer_->lookupTransform(costmap_ros_->getBaseFrameID(),
                                              costmap_ros_->getGlobalFrameID(),
                                              ros::Time(),
                                              ros::Duration(0.5));
      tf2::Stamped<tf2::Transform> tf2_odom_to_base;
      tf2::convert(odom_to_base, tf2_odom_to_base);
      tf2_odom_to_base.setData(tf2_odom_to_base.inverse());
      base_to_odom = tf2::toMsg(tf2_odom_to_base);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR("Could not transform to %s", costmap_ros_->getBaseFrameID().c_str());
      return false;
    }

    // Get the overall goal
    geometry_msgs::PoseStamped goal_pose;
    if (!planner_util_.getGoal(goal_pose))
    {
      ROS_ERROR("Unable to get goal");
      return false;
    }

    // Compute distance to goal
    double dist_to_goal = std::hypot(goal_pose.pose.position.x - robot_pose_.pose.position.x,
                                     goal_pose.pose.position.y - robot_pose_.pose.position.y);

    // If we've reached the XY goal tolerance, just rotate
    if (dist_to_goal < xy_goal_tolerance_ || goal_tolerance_met_)
    {
      // Reached goal, latch if desired
      goal_tolerance_met_ = latch_xy_goal_tolerance_;
      // Compute velocity required to rotate towards goal
      tf2::doTransform(transformed_plan.back(), goal_pose, odom_to_base);
      rotateTowards(goal_pose, cmd_vel);
      // Check for collisions between our current pose and goal
      double yaw_start = tf2::getYaw(robot_pose_.pose.orientation);
      double yaw_end = tf2::getYaw(goal_pose.pose.orientation);
      size_t num_steps = fabs(yaw_end - yaw_start) / 0.1;
      // Need to check at least the end pose
      if (num_steps < 1) num_steps = 1;
      for (size_t i = 1; i <= num_steps; ++i)
      {
        double step = static_cast<double>(i) / static_cast<double>(num_steps);
        double yaw = yaw_start + (step * (yaw_start - yaw_end));
        if (isColliding(robot_pose_.pose.position.x,
                        robot_pose_.pose.position.y,
                        yaw,
                        costmap_ros_))
        {
          ROS_ERROR("Unable to rotate in place due to collision.");
          return false;
        }
      }
      // Safe to rotate, execute computed command
      return true;
    }

    // Work back from the end of plan to find valid target pose
    for (int i = transformed_plan.size() - 1; i >= 0; --i)
    {
      // Underlying control law needs a single target pose, which should:
      //  * Be as far away as possible from the robot (for smoothness)
      //  * But no further than the max_lookahed_ distance
      //  * Be feasible to reach in a collision free manner
      geometry_msgs::PoseStamped target_pose;

      // Transform potential target pose into base_link
      tf2::doTransform(transformed_plan[i], target_pose, odom_to_base);

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
          // Avoid unstability and big sweeping turns at the end of paths by ignoring final heading
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

      // Configure controller max velocity based on current speed
      if (!odom_helper_.getOdomTopic().empty())
      {
        // The API of the OdometryHelperROS uses a PoseStamped
        // but the data returned is velocities (should be a Twist)
        geometry_msgs::PoseStamped robot_velocity;
        odom_helper_.getRobotVel(robot_velocity);
        double max_vel_x = robot_velocity.pose.position.x + (acc_lim_x_ * acc_dt_);
        max_vel_x = std::min(max_vel_x, max_vel_x_);
        max_vel_x = std::max(max_vel_x, min_vel_x_);
        controller_->setVelocityLimits(min_vel_x_, max_vel_x, max_vel_theta_);
      }

      // Simulated path (for debugging/visualization)
      std::vector<geometry_msgs::PoseStamped> simulated_path;
      // Should we simulate rotation initially
      bool sim_initial_rotation_ = has_new_path_ && initial_rotate_tolerance_ > 0.0;
      // Get control and path, iteratively
      while (true)
      {
        // The error between current robot pose and the target pose
        geometry_msgs::PoseStamped error = target_pose;

        // Extract error_angle
        double error_angle = tf2::getYaw(error.pose.orientation);

        // Move origin to our current simulated pose
        if (!simulated_path.empty())
        {
          double x = error.pose.position.x - simulated_path.back().pose.position.x;
          double y = error.pose.position.y - simulated_path.back().pose.position.y;

          double theta = -tf2::getYaw(simulated_path.back().pose.orientation);
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
          geometry_msgs::Twist rotation;
          if (fabs(rotateTowards(error, rotation)) < initial_rotate_tolerance_)
          {
            if (simulated_path.empty())
            {
              // Current robot pose satisifies initial rotate tolerance
              ROS_WARN("Done rotating towards path");
              has_new_path_ = false;
            }
            sim_initial_rotation_ = false;
          }
          vel_x = rotation.linear.x;
          vel_th = rotation.angular.z;
        }

        if (!sim_initial_rotation_)
        {
          if (!controller_->approach(error.pose.position.x, error.pose.position.y, error_angle,
                                     vel_x, vel_th))
          {
            ROS_ERROR("Unable to compute approach");
            return false;
          }
        }

        if (simulated_path.empty())
        {
          // First iteration of simulation, store our commands to the robot
          cmd_vel.linear.x = vel_x;
          cmd_vel.angular.z = vel_th;
        }
        else if (std::hypot(error.pose.position.x, error.pose.position.y) < resolution_)
        {
          // We've simulated to the desired pose, can return this result
          base_local_planner::publishPlan(simulated_path, local_plan_pub_);
          target_pose_pub_.publish(target_pose);
          return true;
        }

        // Forward simulate command
        geometry_msgs::PoseStamped next_pose;
        next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
        if (simulated_path.empty())
        {
          // Initialize at origin
          next_pose.pose.orientation.w = 1.0;
        }
        else
        {
          // Start at last pose
          next_pose = simulated_path.back();
        }

        // Generate next pose
        double dt = (vel_x > 0.0) ?  resolution_ / vel_x : 0.1;
        double yaw = tf2::getYaw(next_pose.pose.orientation);
        next_pose.pose.position.x += dt * vel_x * cos(yaw);
        next_pose.pose.position.y += dt * vel_x * sin(yaw);
        yaw += dt * vel_th;
        next_pose.pose.orientation.z = sin(yaw / 2.0);
        next_pose.pose.orientation.w = cos(yaw / 2.0);
        simulated_path.push_back(next_pose);

        // Check next pose for collision
        tf2::doTransform(next_pose, next_pose, base_to_odom);
        if (isColliding(next_pose.pose.position.x,
                        next_pose.pose.position.y,
                        tf2::getYaw(next_pose.pose.orientation),
                        costmap_ros_))
        {
          // Reason will be printed in function
          break;
        }
      }
    }

    ROS_ERROR("No pose in path was reachable");
    return false;
  }

  /**
   * @brief Check if the goal pose has been achieved by the local planner
   * @return True if achieved, false otherwise
   */
  virtual bool isGoalReached()
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner is not initialized, call initialize() before using this planner");
      return false;
    }

    if (!costmap_ros_->getRobotPose(robot_pose_))
    {
      ROS_ERROR("Could not get the robot pose");
      return false;
    }

    geometry_msgs::PoseStamped goal;
    if (!planner_util_.getGoal(goal))
    {
      ROS_ERROR("Unable to get goal");
      return false;
    }

    double dist = std::hypot(goal.pose.position.x - robot_pose_.pose.position.x,
                             goal.pose.position.y - robot_pose_.pose.position.y);

    double angle = angles::shortest_angular_distance(tf2::getYaw(goal.pose.orientation),
                                                     tf2::getYaw(robot_pose_.pose.orientation));

    return (dist < xy_goal_tolerance_) && (fabs(angle) < yaw_goal_tolerance_);
  }

  /**
   * @brief Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  virtual bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner is not initialized, call initialize() before using this planner");
      return false;
    }

    // We need orientations on our poses
    std::vector<geometry_msgs::PoseStamped> oriented_plan;
    if (compute_orientations_)
    {
      oriented_plan = addOrientations(plan);
    }
    else
    {
      oriented_plan = plan;
    }

    // Filter noisy orientations (if desired)
    std::vector<geometry_msgs::PoseStamped> filtered_plan;
    if (use_orientation_filter_)
    {
      filtered_plan = applyOrientationFilter(oriented_plan, yaw_filter_tolerance_, yaw_gap_tolerance_);
    }
    else
    {
      filtered_plan = oriented_plan;
    }

    // Store the plan for computeVelocityCommands
    if (planner_util_.setPlan(filtered_plan))
    {
      // Reset flags
      has_new_path_ = true;
      goal_tolerance_met_ = false;
      ROS_INFO("Recieved a new path with %lu points", filtered_plan.size());
      return true;
    }

    // Signal error
    return false;
  }

  /**
   * @brief Rotate the robot towards a goal.
   * @param pose The pose should always be in base frame!
   * @param cmd_vel The returned command velocity
   * @returns The computed angular error.
   */
  double rotateTowards(const geometry_msgs::PoseStamped& pose,
                       geometry_msgs::Twist& cmd_vel)
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

    ROS_DEBUG_NAMED("graceful_controller", "Rotating towards goal, error = %f", yaw);

    // Determine max velocity based on current speed
    double max_vel_th = max_vel_theta_;
    if (!odom_helper_.getOdomTopic().empty())
    {
      // The API of the OdometryHelperROS uses a PoseStamped
      // but the data returned is velocities (should be a Twist)
      geometry_msgs::PoseStamped robot_velocity;
      odom_helper_.getRobotVel(robot_velocity);
      double abs_vel = fabs(tf2::getYaw(robot_velocity.pose.orientation));
      double acc_limited = abs_vel + (acc_lim_theta_ * acc_dt_);
      max_vel_th = std::min(max_vel_th, acc_limited);
      max_vel_th = std::max(max_vel_th, min_in_place_vel_theta_);
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 2 * acc_lim_theta_ * fabs(yaw);
    cmd_vel.angular.z = sign(yaw) * std::min(max_vel_th, std::max(min_in_place_vel_theta_, cmd_vel.angular.z));

    // Return error
    return yaw;
  }

private:
  void velocityCallback(const std_msgs::Float32::ConstPtr& max_vel_x)
  {
    // Lock the mutex
    std::lock_guard<std::mutex> lock(config_mutex_);

    max_vel_x_ = std::max(static_cast<double>(max_vel_x->data), min_vel_x_);
  }

  ros::Publisher global_plan_pub_, local_plan_pub_, target_pose_pub_;
  ros::Subscriber max_vel_sub_;

  bool initialized_;
  GracefulControllerPtr controller_;

  tf2_ros::Buffer* buffer_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  base_local_planner::LocalPlannerUtil planner_util_;
  base_local_planner::OdometryHelperRos odom_helper_;

  // Parameters and dynamic reconfigure
  std::mutex config_mutex_;
  dynamic_reconfigure::Server<GracefulControllerConfig> *dsrv_;
  double max_vel_x_;
  double min_vel_x_;
  double max_vel_theta_;
  double min_in_place_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
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

  geometry_msgs::PoseStamped robot_pose_;
};

}  // namespace graceful_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(graceful_controller::GracefulControllerROS, nav_core::BaseLocalPlanner)
