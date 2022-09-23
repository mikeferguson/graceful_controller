/*
 * Copyright 2021-2022 Michael Ferguson
 * Copyright 2015 Fetch Robotics Inc
 * Author: Michael Ferguson
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GRACEFUL_CONTROLLER_HPP
#define GRACEFUL_CONTROLLER_HPP

#include <memory>
#include <vector>

namespace graceful_controller
{

class GracefulController
{
public:
  /**
   * @brief Constructor of the controller.
   * @param k1 Ratio of rate of change of theta to rate of change of R.
   * @param k2 How quickly we converge to the slow manifold.
   * @param min_abs_velocity The minimum absolute velocity in the linear direction.
   * @param max_abs_velocity The maximum absolute velocity in the linear direction.
   * @param max_decel The maximum deceleration in the linear direction.
   * @param max_abs_angular_velocity The maximum absolute velocity in the angular direction.
   */
  GracefulController(double k1,
                     double k2,
                     double min_abs_velocity,
                     double max_abs_velocity,
                     double max_decel,
                     double max_abs_angular_velocity,
                     double beta,
                     double lambda);

  /**
   * @brief Implements something loosely based on "A Smooth Control Law for
   * Graceful Motion of Differential Wheeled Mobile Robots in 2D Environments"
   * by Park and Kuipers, ICRA 2011
   * @param x The x coordinate of the goal, relative to robot base link.
   * @param y The y coordinate of the goal, relative to robot base link.
   * @param theta The angular orientation of the goal, relative to robot base link.
   * @param vel_x The computed command velocity in the linear direction.
   * @param vel_th The computed command velocity in the angular direction.
   * @param backward_motion Flag to indicate that the robot should move backward. False by default.
   * @returns true if there is a solution.
   */
  bool approach(const double x, const double y, const double theta,
                double& vel_x, double& vel_th, bool backward_motion=false);

  /**
   * @brief Update the velocity limits.
   * @param min_abs_velocity The minimum absolute velocity in the linear direction.
   * @param max_abs_velocity The maximum absolute velocity in the linear direction.
   * @param max_abs_angular_velocity The maximum absolute velocity in the angular direction.
   */
  void setVelocityLimits(const double min_abs_velocity,
                         const double max_abs_velocity,
                         const double max_abs_angular_velocity);

private:
  /*
   * Parameters for approach controller
   */
  double k1_;  // ratio in change of theta to rate of change in r
  double k2_;  // speed at which we converge to slow system
  double min_abs_velocity_;
  double max_abs_velocity_;
  double max_decel_;
  double max_abs_angular_velocity_;
  double beta_;  // how fast velocity drops as k increases
  double lambda_; // controls speed scaling based on curvature. A higher value of lambda results in more sharply peaked curves
  double dist_;  // used to create the tracking line
};

using GracefulControllerPtr = std::shared_ptr<GracefulController>;

}  // namespace graceful_controller

#endif  // GRACEFUL_CONTROLLER_HPP
