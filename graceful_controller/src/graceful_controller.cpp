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

#include <graceful_controller/graceful_controller.hpp>

#include <angles/angles.h>

#include <algorithm>
#include <list>
#include <vector>
#include <cmath>

namespace graceful_controller
{

GracefulController::GracefulController(double k1, double k2,
                                       double min_abs_velocity, double max_abs_velocity,
                                       double max_decel,
                                       double max_abs_angular_velocity,
                                       double beta, double lambda)
{
  k1_ = k1;
  k2_ = k2;
  min_abs_velocity_ = min_abs_velocity;
  max_abs_velocity_ = max_abs_velocity;
  max_decel_ = max_decel;
  max_abs_angular_velocity_ = max_abs_angular_velocity;
  beta_ = beta;
  lambda_ = lambda;
}

// x, y, theta are relative to base location and orientation
bool GracefulController::approach(const double x, const double y, const double theta,
                                  double& vel_x, double& vel_th, bool backward_motion)
{
  // Distance to goal
  double r = std::sqrt(x * x + y * y);

  // Orientation base frame relative to r_
  double delta = (backward_motion) ? std::atan2(-y, -x) : std::atan2(-y, x);

  // Determine orientation of goal frame relative to r_
  double theta2 = angles::normalize_angle(theta + delta);

  // Compute the virtual control
  double a = std::atan(-k1_ * theta2);
  // Compute curvature (k)
  double k = -1.0/r * (k2_ * (delta - a) + (1 + (k1_/(1+((k1_*theta2)*(k1_*theta2)))))*sin(delta));

  // Compute max_velocity based on curvature
  double v = max_abs_velocity_ / (1 + beta_ * std::pow(fabs(k), lambda_));
  // Limit velocity based on approaching target
  double approach_limit = std::sqrt(2 * max_decel_ * r);
  v = std::min(v, approach_limit);
  v = std::min(std::max(v, min_abs_velocity_), max_abs_velocity_);
  if (backward_motion)
  {
    v *= -1; // reverse linear velocity direction for backward motion
  }

  // Compute angular velocity
  double w = k * v;
  // Bound angular velocity
  double bounded_w = std::min(max_abs_angular_velocity_, std::max(-max_abs_angular_velocity_, w));
  // Make sure that if we reduce w, we reduce v so that kurvature is still followed
  if (w != 0.0)
  {
    v *= (bounded_w/w);
  }

  // Send command to base
  vel_x = v;
  vel_th = bounded_w;
  return true;
}

void GracefulController::setVelocityLimits(
  const double min_abs_velocity,
  const double max_abs_velocity,
  const double max_abs_angular_velocity)
{
  min_abs_velocity_ = min_abs_velocity;
  max_abs_velocity_ = max_abs_velocity;
  max_abs_angular_velocity_ = max_abs_angular_velocity;
}

}  // namespace graceful_controller
