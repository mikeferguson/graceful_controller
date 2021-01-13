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
* Author: Eitan Marder-Eppstein, Michael Ferguson
*********************************************************************/

#include <cmath>
#include <mutex>

#include <angles/angles.h>
#include <graceful_controller/graceful_controller.hpp>
#include <nav_2d_utils/parameters.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_costmap_2d/footprint.h>
#include <nav2_util/line_iterator.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using rclcpp_lifecycle::LifecyclePublisher;
using nav2_util::declare_parameter_if_not_declared;

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
                 nav2_costmap_2d::Costmap2DROS* costmap)
{
  unsigned mx, my;
  if (!costmap->getCostmap()->worldToMap(x, y, mx, my))
  {
    ROS_DEBUG("Path is off costmap (%f,%f)", x, y);
    return true;
  }

  std::vector<geometry_msgs::Point> spec = costmap->getRobotFootprint();
  std::vector<geometry_msgs::Point> footprint;
  nav2_costmap_2d::transformFootprint(x, y, theta, spec, footprint);

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

    for (nav2_util::LineIterator line(x0,y0,x1,y1); line.isValid(); line.advance())
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

class GracefulControllerROS : public nav2_core::Controller
{
public:
  GracefulControllerROS() :
    has_new_path_(false)
  {
  }

  /**
   * @brief Constructs the local planner
   * @param name The name to give this instance of the local planner
   * @param tf A pointer to a transform buffer
   * @param costmap_ros The cost map to use for assigning costs to local plans
   */
  virtual void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
                         std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
                         const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
  {
    if (!initialized_)
    {
      node_ = node;
      clock_ = node->get_clock();
      logger_ = node->get_logger();
      buffer_ = tf;
      costmap_ros_ = costmap_ros;
      name_ = name;

      // Publishers (same topics as DWA/TrajRollout)
      global_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("global_plan", 1);
      local_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

      declare_parameter_if_not_declared(node_, name_ + ".max_vel_x");
      declare_parameter_if_not_declared(node_, name_ + ".min_vel_x");
      declare_parameter_if_not_declared(node_, name_ + ".max_vel_theta");
      declare_parameter_if_not_declared(node_, name_ + ".min_vel_theta");
      declare_parameter_if_not_declared(node_, name_ + ".min_in_place_vel_theta");
      declare_parameter_if_not_declared(node_, name_ + ".acc_lim_x");
      declare_parameter_if_not_declared(node_, name_ + ".acc_lim_theta");
      declare_parameter_if_not_declared(node_, name_ + ".acc_dt", rclcpp::ParameterValue(0.25));
      declare_parameter_if_not_declared(node_, name_ + ".xy_goal_tolerance");
      declare_parameter_if_not_declared(node_, name_ + ".max_lookahead", rclcpp::ParameterValue(1.0));
      declare_parameter_if_not_declared(node_, name_ + ".initial_rotate_tolerance", rclcpp::ParameterValue(0.1));
      declare_parameter_if_not_declared(node_, name_ + ".use_vel_topic", rclcpp::ParameterValue(false));
      declare_parameter_if_not_declared(node_, name_ + ".k1", rclcpp::ParameterValue(2.0));
      declare_parameter_if_not_declared(node_, name_ + ".k2", rclcpp::ParameterValue(1.0));
      declare_parameter_if_not_declared(node_, name_ + ".beta", rclcpp::ParameterValue(0.4));
      declare_parameter_if_not_declared(node_, name_ + ".lambda", rclcpp::ParameterValue(2.0));

      node_->get_parameter(name_ + ".max_vel_x", max_vel_x_);
      node_->get_parameter(name_ + ".min_vel_x", min_vel_x_);
      node_->get_parameter(name_ + ".max_vel_theta", max_vel_theta_);
      node_->get_parameter(name_ + ".min_vel_theta", min_vel_theta_);
      node_->get_parameter(name_ + ".min_in_place_vel_theta", min_in_place_vel_theta_);
      node_->get_parameter(name_ + ".acc_lim_x", acc_lim_x_);
      node_->get_parameter(name_ + ".acc_lim_theta", acc_lim_theta_);
      node_->get_parameter(name_ + ".acc_dt", acc_dt_);
      node_->get_parameter(name_ + ".xy_goal_tolerance", xy_goal_tolerance_);
      node_->get_parameter(name_ + ".max_lookahead", max_lookahead_);
      node_->get_parameter(name_ + ".initial_rotate_tolerance", initial_rotate_tolerance_);
      resolution_ = costmap_ros_->getCostmap()->getResolution();
      double k1, k2, beta, lambda;
      node_->get_parameter(name_ + ".k1", k1);
      node_->get_parameter(name_ + ".k2", k2);
      node_->get_parameter(name_ + ".beta", beta);
      node_->get_parameter(name_ + ".lambda", lambda);

      bool use_vel_topic = false;
      node_->get_parameter(name_ + ".use_vel_topic", use_vel_topic);
      if (use_vel_topic)
      {
        max_vel_sub_ = node_->create_subscription<std_msgs::msg::Float32>("max_vel_x", 1, std::bind(&GracefulControllerROS::velocityCallback, this, std::placeholders::_1));
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
    else
    {
      RCLCPP_ERROR(logger_, "This planner has already been initialized, doing nothing.");
    }
  }

  void activate()
  {
    global_plan_pub_->on_activate();
    local_plan_pub_->on_activate();
    has_new_path_ = false;
  }

  void deactivate()
  {
    global_plan_pub_->on_deactivate();
    local_plan_pub_->on_deactivate();
  }

  void cleanup()
  {
    global_plan_pub_.reset();
    global_plan_pub_.reset();
  }

  /**
   * @brief Controller computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * @param robot_pose Current robot pose
   * @param velocity Current robot velocity
   * @return The best command for the robot to drive
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    const geometry_msgs::msg::Twist& velocity)
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = robot_pose.header.frame_id;
    cmd_vel.header.stamp = clock_->now();

    if (!initialized_)
    {
      RCLCPP_ERROR(logger_, "Controller is not initialized, call configure() before using this planner");
      return cmd_vel;
    }

    // Lock the mutex
    std::lock_guard<std::mutex> lock(config_mutex_);

    // Get transforms
    geometry_msgs::msg::TransformStamped plan_to_base, base_to_odom;
    try
    {
      plan_to_base = buffer_->lookupTransform(costmap_ros_->getBaseFrameID(),
                                              global_plan_.header.frame_id,
                                              tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(logger_, "Could not transform to %s", costmap_ros_->getBaseFrameID().c_str());
      return cmd_vel;
    }

    try
    {
      base_to_odom = buffer_->lookupTransform(costmap_ros_->getGlobalFrameID(),
                                              costmap_ros_->getBaseFrameID(),
                                              tf2::TimePointZero);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(logger_, "Could not transform to %s", costmap_ros_->getGlobalFrameID().c_str());
      return cmd_vel;
    }

    geometry_msgs::msg::PoseStamped pose;
    tf2::doTransform(global_plan_.poses.back(), pose, plan_to_base);
    if (std::hypot(pose.pose.position.x, pose.pose.position.y) < xy_goal_tolerance_)
    {
      // XY goal tolerance reached - now just rotate towards goal
      rotateTowards(pose, velocity, cmd_vel);
      return cmd_vel;
    }

    // Work back from the end of plan
    for (int i = global_plan_.poses.size() - 1; i >= 0; --i)
    {
      // Transform pose into base_link
      tf2::doTransform(global_plan_.poses[i], pose, plan_to_base);

      // Continue if this is too far away
      if (std::hypot(pose.pose.position.x, pose.pose.position.y) > max_lookahead_)
      {
        continue;
      }

      // Configure controller max velocity based on current speed
      if (acc_dt_ > 0.0)
      {
        double max_vel_x = velocity.linear.x + (acc_lim_x_ * acc_dt_);
        max_vel_x = std::min(max_vel_x, max_vel_x_);
        max_vel_x = std::max(max_vel_x, min_vel_x_);
        controller_->setVelocityLimits(min_vel_x_, max_vel_x, max_vel_theta_);
      }

      // Simulated path (for debugging/visualization)
      nav_msgs::msg::Path path;
      // Get control and path, iteratively
      while (true)
      {
        // The error between current robot pose and the lookahead goal
        geometry_msgs::msg::PoseStamped error = pose;

        // Extract error_angle
        double error_angle = tf2::getYaw(error.pose.orientation);

        // Move origin to our current simulated pose
        if (!path.poses.empty())
        {
          double x = error.pose.position.x - path.poses.back().pose.position.x;
          double y = error.pose.position.y - path.poses.back().pose.position.y;

          double theta = -tf2::getYaw(path.poses.back().pose.orientation);
          error.pose.position.x = x * cos(theta) - y * sin(theta);
          error.pose.position.y = y * cos(theta) + x * sin(theta);

          error_angle += theta;
          error.pose.orientation.z = sin(error_angle / 2.0);
          error.pose.orientation.w = cos(error_angle / 2.0);
        }

        // Compute commands
        double vel_x, vel_th;
        if (!controller_->approach(error.pose.position.x, error.pose.position.y, error_angle,
                                   vel_x, vel_th))
        {
          RCLCPP_ERROR(logger_, "Unable to compute approach");
          return cmd_vel;
        }

        if (path.poses.empty())
        {
          cmd_vel.twist.linear.x = vel_x;
          cmd_vel.twist.angular.z = vel_th;
        }
        else if (std::hypot(error.pose.position.x, error.pose.position.y) < resolution_)
        {
          if (has_new_path_ && initial_rotate_tolerance_ > 0.0)
          {
            // Rotate towards goal
            geometry_msgs::msg::TwistStamped rotation;
            if (fabs(rotateTowards(pose, velocity, rotation)) < initial_rotate_tolerance_)
            {
              RCLCPP_INFO(logger_, "Done rotating towards path");
              has_new_path_ = false;
            }
            else
            {
              cmd_vel.twist = rotation.twist;
            }
          }

          // We have reached lookahead goal without collision
          local_plan_pub_->publish(path);
          return cmd_vel;
        }

        // Forward simulate command
        geometry_msgs::msg::PoseStamped next_pose;
        next_pose.header.frame_id = costmap_ros_->getBaseFrameID();
        if (path.poses.empty())
        {
          // Initialize at origin
          next_pose.pose.orientation.w = 1.0;
        }
        else
        {
          // Start at last pose
          next_pose = path.poses.back();
        }

        // Generate next pose
        double dt = resolution_ / vel_x;
        double yaw = tf2::getYaw(next_pose.pose.orientation);
        next_pose.pose.position.x += dt * vel_x * cos(yaw);
        next_pose.pose.position.y += dt * vel_x * sin(yaw);
        yaw += dt * vel_th;
        next_pose.pose.orientation.z = sin(yaw / 2.0);
        next_pose.pose.orientation.w = cos(yaw / 2.0);
        path.poses.push_back(next_pose);

        // Check next pose for collision
        tf2::doTransform(next_pose, next_pose, base_to_odom);
        if (isColliding(next_pose.pose.position.x,
                        next_pose.pose.position.y,
                        yaw,
                        costmap_ros_))
        {
          // Reason will be printed in function
          break;
        }
      }
    }

    RCLCPP_ERROR(logger_, "No pose in path was reachable");
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  /**
   * @brief Set the plan that the local planner is following
   * @param plan The plan to pass to the local planner
   * @return True if the plan was updated successfully, false otherwise
   */
  virtual void setPlan(const nav_msgs::msg::Path& path) override
  {
    // We need orientations on our poses
    nav_msgs::msg::Path oriented_plan;
    oriented_plan.header = path.header;
    oriented_plan.poses.resize(path.poses.size());

    // Copy the only oriented pose
    oriented_plan.poses.back() = path.poses.back();

    // For each pose, point at the next one
    for (size_t i = 0; i < oriented_plan.poses.size() - 1; ++i)
    {
      oriented_plan.poses[i] = path.poses[i];
      double dx = path.poses[i+1].pose.position.x - path.poses[i].pose.position.x;
      double dy = path.poses[i+1].pose.position.y - path.poses[i].pose.position.y;
      double yaw = std::atan2(dy, dx);
      oriented_plan.poses[i].pose.orientation.z = sin(yaw / 2.0);
      oriented_plan.poses[i].pose.orientation.w = cos(yaw / 2.0);
    }

    // TODO: filter noisy orientations?

    global_plan_ = oriented_plan;
    has_new_path_ = true;
    RCLCPP_INFO(logger_, "Recieved a new path with %lu points", oriented_plan.poses.size());
  }

  /**
   * @brief Rotate the robot towards a goal.
   * @param pose The pose should always be in base frame!
   * @param velocity The current robot velocity
   * @param cmd_vel The returned command velocity
   * @returns The computed angular error.
   */
  double rotateTowards(const geometry_msgs::msg::PoseStamped& pose,
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

    RCLCPP_DEBUG(logger_, "Rotating towards goal, error = %f", yaw);

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

private:
  void velocityCallback(const std_msgs::msg::Float32::SharedPtr max_vel_x)
  {
    // Lock the mutex
    std::lock_guard<std::mutex> lock(config_mutex_);

    max_vel_x_ = std::max(static_cast<double>(max_vel_x->data), min_vel_x_);
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("GracefulController")};
  std::string name_;

  std::shared_ptr<LifecyclePublisher<nav_msgs::msg::Path>> global_plan_pub_;
  std::shared_ptr<LifecyclePublisher<nav_msgs::msg::Path>> local_plan_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr max_vel_sub_;

  bool initialized_;
  GracefulControllerPtr controller_;

  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Parameters
  std::mutex config_mutex_;
  double max_vel_x_;
  double min_vel_x_;
  double max_vel_theta_;
  double min_vel_theta_;
  double min_in_place_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  double xy_goal_tolerance_;
  double max_lookahead_;
  double resolution_;
  double acc_dt_;

  // Controls initial rotation towards path
  double initial_rotate_tolerance_;
  bool has_new_path_;

  nav_msgs::msg::Path global_plan_;
};

}  // namespace graceful_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(graceful_controller::GracefulControllerROS, nav2_core::Controller)
