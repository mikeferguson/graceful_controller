/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2020, Michael Ferguson
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.hpp>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/utils.h>

class ControllerFixture
{
public:
  ControllerFixture() :
    loader_("nav_core", "nav_core::BaseLocalPlanner"),
    listener_(buffer_),
    costmap_ros_(NULL),
    shutdown_(false)
  {
  }

  bool setup()
  {
    ros::NodeHandle nh("~");

    // ROS topics to run the test
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true /* latch */);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    max_vel_pub_ = nh.advertise<std_msgs::Float32>("/max_vel_x", 1, true /* latch */);

    // Need to start publishing odom before we initialize the costmap
    resetMap();
    resetPose();
    thread_ = new boost::thread(boost::bind(&ControllerFixture::updateThread, this));

    // Costmap for testing
    costmap_ros_ = new costmap_2d::Costmap2DROS("costmap", buffer_);

    // Controller instance
    try
    {
      controller_ = loader_.createInstance("graceful_controller/GracefulControllerROS");
      controller_->initialize(loader_.getName("graceful_controller/GracefulControllerROS"),
                              &buffer_, costmap_ros_);
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the controller. Exception: %s", ex.what());
      return false;
    }

    return true;
  }

  ~ControllerFixture()
  {
    shutdown_ = true;
    thread_->interrupt();
    thread_->join();
    delete thread_;
  }

  boost::shared_ptr<nav_core::BaseLocalPlanner> getController()
  {
    return controller_;
  }

  costmap_2d::Costmap2DROS * getCostmap()
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

  void setMaxVelocity(float velocity)
  {
    std_msgs::Float32 msg;
    msg.data = velocity;
    max_vel_pub_.publish(msg);
  }

  void setSimCommand(geometry_msgs::Twist& command)
  {
    command_ = command;
  }

protected:
  void updateThread()
  {
    map_pub_.publish(map_);

    ros::Rate r(20.0);
    while (ros::ok() && !shutdown_)
    {
      // Propagate position
      double yaw = tf2::getYaw(odom_.pose.pose.orientation);
      odom_.pose.pose.position.x += 0.05 * command_.linear.x * cos(yaw);
      odom_.pose.pose.position.y += 0.05 * command_.linear.x * sin(yaw);
      yaw += 0.05 * command_.angular.z;
      odom_.pose.pose.orientation.z = sin(yaw / 2.0);
      odom_.pose.pose.orientation.w = cos(yaw / 2.0);

      // Publish updated odom
      odom_.header.stamp = ros::Time::now();
      odom_pub_.publish(odom_);

      // Fake localization
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = ros::Time::now();
      transform.header.frame_id = "map";
      transform.child_frame_id = odom_.header.frame_id;
      transform.transform.rotation.w = 1.0;
      broadcaster_.sendTransform(transform);

      // Publish updated odom as TF
      transform.header.frame_id = odom_.header.frame_id;
      transform.child_frame_id = odom_.child_frame_id;
      transform.transform.translation.x = odom_.pose.pose.position.x;
      transform.transform.translation.y = odom_.pose.pose.position.y;
      transform.transform.rotation = odom_.pose.pose.orientation;
      broadcaster_.sendTransform(transform);

      ros::spinOnce();
      r.sleep();
    }
  }

  pluginlib::ClassLoader<nav_core::BaseLocalPlanner> loader_;
  boost::shared_ptr<nav_core::BaseLocalPlanner> controller_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  ros::Publisher map_pub_, odom_pub_, max_vel_pub_;
  nav_msgs::OccupancyGrid map_;
  nav_msgs::Odometry odom_;
  geometry_msgs::Twist command_;
  boost::thread* thread_;
  bool shutdown_;
};

TEST(ControllerTests, test_basic_plan)
{
  ControllerFixture fixture;
  ASSERT_TRUE(fixture.setup());
  boost::shared_ptr<nav_core::BaseLocalPlanner> controller = fixture.getController();

  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0.0;
  plan.push_back(pose);
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 0.0;
  plan.push_back(pose);
  EXPECT_TRUE(controller->setPlan(plan));

  // Expect max velocity forward
  geometry_msgs::Twist command;
  EXPECT_TRUE(controller->computeVelocityCommands(command));
  EXPECT_EQ(command.linear.x, 1.0);
  EXPECT_EQ(command.angular.z, 0.0);

  // Set a new max velocity by topic
  fixture.setMaxVelocity(0.5);
  ros::Duration(0.25).sleep();

  EXPECT_TRUE(controller->computeVelocityCommands(command));
  EXPECT_EQ(command.linear.x, 0.5);
  EXPECT_EQ(command.angular.z, 0.0);
}

TEST(ControllerTests, test_out_of_range)
{
  ControllerFixture fixture;
  ASSERT_TRUE(fixture.setup());
  boost::shared_ptr<nav_core::BaseLocalPlanner> controller = fixture.getController();

  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  pose.pose.position.x = 1.5;
  pose.pose.position.y = 0.0;
  plan.push_back(pose);
  EXPECT_TRUE(controller->setPlan(plan));

  // First pose is beyond the lookahead - should fail
  geometry_msgs::Twist command;
  EXPECT_FALSE(controller->computeVelocityCommands(command));
}

TEST(ControllerTests, test_rotate_in_place)
{
  ControllerFixture fixture;
  ASSERT_TRUE(fixture.setup());
  boost::shared_ptr<nav_core::BaseLocalPlanner> controller = fixture.getController();

  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  pose.pose.position.x = -0.5;
  pose.pose.position.y = 0.0;
  plan.push_back(pose);
  pose.pose.position.x = -1.0;
  pose.pose.position.y = 0.0;
  plan.push_back(pose);
  EXPECT_TRUE(controller->setPlan(plan));

  // Expect max rotation
  geometry_msgs::Twist command;
  EXPECT_TRUE(controller->computeVelocityCommands(command));
  EXPECT_EQ(command.linear.x, 0.0);
  EXPECT_EQ(command.angular.z, 2.5);
}

TEST(ControllerTests, test_collision_check)
{
  ControllerFixture fixture;
  ASSERT_TRUE(fixture.setup());
  boost::shared_ptr<nav_core::BaseLocalPlanner> controller = fixture.getController();

  fixture.markMap(0.65, 0);

  std::vector<geometry_msgs::PoseStamped> plan;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.pose.orientation.w = 1.0;
  pose.pose.position.x = 0.5;
  pose.pose.position.y = 0.0;
  plan.push_back(pose);
  EXPECT_TRUE(controller->setPlan(plan));

  // Expect max rotation
  geometry_msgs::Twist command;
  EXPECT_FALSE(controller->computeVelocityCommands(command));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "graceful_controller_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
