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

#include <gtest/gtest.h>
#include <nav2_core/controller.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_loader.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("graceful_controller_tests");

class ControllerFixture : public rclcpp_lifecycle::LifecycleNode
{
public:
  ControllerFixture() :
    LifecycleNode("graceful_conroller_tests"),
    loader_("nav_core", "nav_core::BaseLocalPlanner"),
    costmap_ros_(NULL),
    shutdown_(false)
  {
    buffer_.reset(new tf2_ros::Buffer(this->get_clock()));
    listener_.reset(new tf2_ros::TransformListener(*buffer_));
    broadcaster_.reset(new tf2_ros::TransformBroadcaster(this));
  }

  bool setup(bool intialize = true)
  {
    // ROS topics to run the test
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());

    // Need to start publishing odom before we initialize the costmap
    resetMap();
    resetPose();
    thread_ = new std::thread(std::bind(&ControllerFixture::updateThread, this));

    // Costmap for testing
    costmap_ros_.reset(new nav2_costmap_2d::Costmap2DROS("costmap"));

    // Controller instance
    try
    {
      controller_ = loader_.createSharedInstance("graceful_controller/GracefulControllerROS");
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(LOGGER, "Failed to create the controller. Exception: %s", ex.what());
      return false;
    }

    if (intialize)
    {
      initialize();
    }

    return true;
  }

  ~ControllerFixture()
  {
    // Stop costmap updates - otherwise this can fail with a boost::lock issue
    costmap_ros_->stop();
    // Stop our own update thread
    shutdown_ = true;
    thread_->join();
    delete thread_;
  }

  void initialize()
  {
    controller_->configure(this->shared_from_this(),
                           loader_.getName("graceful_controller/GracefulControllerROS"),
                           buffer_,
                           costmap_ros_);
  }

  std::shared_ptr<nav2_core::Controller> getController()
  {
    return controller_;
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getCostmap()
  {
    return costmap_ros_;
  }

  void resetMap()
  {
    map_.header.frame_id = "map";
    map_.info.resolution = 0.05;
    map_.info.width = 100;
    map_.info.height = 100;
    map_.info.origin.position.x = -2.5;
    map_.info.origin.position.y = -2.5;

    map_.data.resize(100 * 100);
    for (size_t i = 0; i < map_.data.size(); ++i)
    {
      map_.data[i] = 0;
    }
  }

  bool markMap(double x, double y)
  {
    int ix = (x - map_.info.origin.position.x) / map_.info.resolution;
    int iy = (y - map_.info.origin.position.y) / map_.info.resolution;

    if (ix < 0 || ix >= static_cast<int>(map_.info.width) ||
        iy < 0 || iy >= static_cast<int>(map_.info.height))
    {
      return false;
    }

    map_.data[ix + (iy * map_.info.width)] = 100;
    map_pub_->publish(map_);
    return true;
  }

  void resetPose()
  {
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_link";
    odom_.pose.pose.position.x = 0.0;
    odom_.pose.pose.position.y = 0.0;
    odom_.pose.pose.orientation.w = 1.0;
  }

  void setPose(double x, double y, double yaw)
  {
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_link";
    odom_.pose.pose.position.x = x;
    odom_.pose.pose.position.y = y;
    odom_.pose.pose.orientation.z = sin(yaw / 2.0);
    odom_.pose.pose.orientation.w = cos(yaw / 2.0);
  }

  void setMaxVelocity(float velocity)
  {
    controller_->setSpeedLimit(velocity, false);
  }

  void setSimVelocity(double x, double th)
  {
    odom_.twist.twist.linear.x = x;
    odom_.twist.twist.angular.z = th;
  }

  void setSimCommand(geometry_msgs::msg::Twist& command)
  {
    command_ = command;
  }

  geometry_msgs::msg::PoseStamped getRobotPose()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom_.header;
    pose.pose = odom_.pose.pose;
    return pose;
  }

  geometry_msgs::msg::Twist getRobotVelocity()
  {
    return odom_.twist.twist;
  }

protected:
  void updateThread()
  {
    map_pub_->publish(map_);

    while (rclcpp::ok() && !shutdown_)
    {
      // Propagate position
      double yaw = tf2::getYaw(odom_.pose.pose.orientation);
      odom_.pose.pose.position.x += 0.05 * command_.linear.x * cos(yaw);
      odom_.pose.pose.position.y += 0.05 * command_.linear.x * sin(yaw);
      yaw += 0.05 * command_.angular.z;
      odom_.pose.pose.orientation.z = sin(yaw / 2.0);
      odom_.pose.pose.orientation.w = cos(yaw / 2.0);

      // Fake localization
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = "map";
      transform.child_frame_id = odom_.header.frame_id;
      transform.transform.rotation.w = 1.0;
      broadcaster_->sendTransform(transform);

      // Publish updated odom as TF
      transform.header.frame_id = odom_.header.frame_id;
      transform.child_frame_id = odom_.child_frame_id;
      transform.transform.translation.x = odom_.pose.pose.position.x;
      transform.transform.translation.y = odom_.pose.pose.position.y;
      transform.transform.rotation = odom_.pose.pose.orientation;
      broadcaster_->sendTransform(transform);

      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(50));
    }
  }

  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  std::shared_ptr<nav2_core::Controller> controller_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> map_pub_;
  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::Odometry odom_;
  geometry_msgs::msg::Twist command_;
  std::thread* thread_;
  bool shutdown_;
};

TEST(ControllerTests, test_basic_plan)
{
  ControllerFixture fixture;
  ASSERT_TRUE(fixture.setup(false /* do not intialize */));
  std::shared_ptr<nav2_core::Controller> controller = fixture.getController();

  nav_msgs::msg::Path plan;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;
  plan.poses.push_back(pose);
  pose.pose.position.x = 1.5;
  pose.pose.position.y = 0.0;
  plan.poses.push_back(pose);

  geometry_msgs::msg::PoseStamped robot_pose = fixture.getRobotPose();
  geometry_msgs::msg::Twist robot_velocity = fixture.getRobotVelocity(); 

  // Unitialized controller should not be able to plan
  controller->setPlan(plan);
  geometry_msgs::msg::TwistStamped command;
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.0);
  EXPECT_EQ(command.twist.angular.z, 0.0);
  fixture.initialize();

  // Calling multiple times should not be a problem
  fixture.initialize();

  // Now we can set the plan
  controller->setPlan(plan);

  // Set velocity to 0
  fixture.setSimVelocity(0.0, 0.0);
  robot_velocity = fixture.getRobotVelocity();

  // Odom reports velocity = 0, but min_vel_x is greater than acc_lim * sacc_dt
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.25);
  EXPECT_EQ(command.twist.angular.z, 0.0);

  // Set a new max velocity
  fixture.setMaxVelocity(0.5);
  robot_velocity = fixture.getRobotVelocity();

  // Odom still reports 0, so max remains the same
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.25);
  EXPECT_EQ(command.twist.angular.z, 0.0);

  // Now lie about velocity
  fixture.setSimVelocity(1.0, 0.0);
  robot_velocity = fixture.getRobotVelocity();

  // Odom now reports 1.0, but max_vel_x topic is 0.5
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.5);
  EXPECT_EQ(command.twist.angular.z, 0.0);

  // Bump our current speed up
  fixture.setMaxVelocity(1.0);
  robot_velocity = fixture.getRobotVelocity();

  // Expect max velocity
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 1.0);
  EXPECT_EQ(command.twist.angular.z, 0.0);

  // Report velocity over limits
  fixture.setSimVelocity(3.0, 0.0);
  robot_velocity = fixture.getRobotVelocity();

  // Expect max velocity
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 1.0);
  EXPECT_EQ(command.twist.angular.z, 0.0);
}

TEST(ControllerTests, test_out_of_range)
{
  ControllerFixture fixture;
  ASSERT_TRUE(fixture.setup());
  std::shared_ptr<nav2_core::Controller> controller = fixture.getController();

  nav_msgs::msg::Path plan;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  pose.pose.position.x = 1.5;
  pose.pose.position.y = 0.0;
  plan.poses.push_back(pose);
  controller->setPlan(plan);

  geometry_msgs::msg::PoseStamped robot_pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.w = 1.0;

  geometry_msgs::msg::Twist robot_velocity;
  robot_velocity.linear.x = 0.0;
  robot_velocity.linear.y = 0.0;

  // First pose is beyond the lookahead - should fail
  geometry_msgs::msg::TwistStamped command;
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.0);
  EXPECT_EQ(command.twist.angular.z, 0.0);
}

TEST(ControllerTests, test_initial_rotate_in_place)
{
  ControllerFixture fixture;
  ASSERT_TRUE(fixture.setup());
  std::shared_ptr<nav2_core::Controller> controller = fixture.getController();

  nav_msgs::msg::Path plan;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  pose.pose.position.x = -0.5;
  pose.pose.position.y = 0.0;
  plan.poses.push_back(pose);
  pose.pose.position.x = -1.0;
  pose.pose.position.y = 0.0;
  plan.poses.push_back(pose);
  controller->setPlan(plan);

  // Set our velocity to 0
  fixture.setSimVelocity(0.0, 0.0);
  geometry_msgs::msg::PoseStamped robot_pose = fixture.getRobotPose();
  geometry_msgs::msg::Twist robot_velocity = fixture.getRobotVelocity();

  // Odom reports velocity = 0, but min_in_place_vel_theta is 0.6
  geometry_msgs::msg::TwistStamped command;
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.0);
  EXPECT_EQ(command.twist.angular.z, 0.6);

  // Set our velocity to 1.0
  fixture.setSimVelocity(0.0, 1.0);
  robot_velocity = fixture.getRobotVelocity();

  // Expect limited rotation command
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.0);
  EXPECT_EQ(command.twist.angular.z, 1.25);

  // Report velocity over limits
  fixture.setSimVelocity(0.0, 4.0);
  robot_velocity = fixture.getRobotVelocity();

  // Expect max rotation
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.0);
  EXPECT_EQ(command.twist.angular.z, 2.5);
}

TEST(ControllerTests, test_final_rotate_in_place)
{
  ControllerFixture fixture;
  ASSERT_TRUE(fixture.setup());
  std::shared_ptr<nav2_core::Controller> controller = fixture.getController();

  nav_msgs::msg::Path plan;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0.0;
  plan.poses.push_back(pose);
  controller->setPlan(plan);

  // Set pose to start
  fixture.setPose(0.1, 0.0, 0.0);
  geometry_msgs::msg::PoseStamped robot_pose = fixture.getRobotPose();
  geometry_msgs::msg::Twist robot_velocity = fixture.getRobotVelocity();

  // Expect forward motion at min velocity since we are aligned with only goal pose
  geometry_msgs::msg::TwistStamped command;
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.25);
  EXPECT_EQ(command.twist.angular.z, 0.0);

  // Set our pose to the end goal, but with bad heading
  fixture.setPose(0.5, 0.0, 1.57);
  robot_pose = fixture.getRobotPose();

  // Expect limited rotation command
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.0);
  EXPECT_EQ(command.twist.angular.z, -0.6);
}

TEST(ControllerTests, test_collision_check)
{
  ControllerFixture fixture;
  ASSERT_TRUE(fixture.setup());
  std::shared_ptr<nav2_core::Controller> controller = fixture.getController();

  fixture.markMap(0.65, 0);
  rclcpp::sleep_for(std::chrono::milliseconds(250));

  nav_msgs::msg::Path plan;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0.0;
  plan.poses.push_back(pose);
  controller->setPlan(plan);

  geometry_msgs::msg::PoseStamped robot_pose = fixture.getRobotPose();
  geometry_msgs::msg::Twist robot_velocity = fixture.getRobotVelocity();

  // Expect no command
  geometry_msgs::msg::TwistStamped command;
  command = controller->computeVelocityCommands(robot_pose, robot_velocity, NULL);
  EXPECT_EQ(command.twist.linear.x, 0.0);
  EXPECT_EQ(command.twist.angular.z, 0.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
