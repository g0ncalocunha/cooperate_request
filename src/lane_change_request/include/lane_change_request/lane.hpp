// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAJECTORY_REMMAPER__NODE_HPP_
#define TRAJECTORY_REMMAPER__NODE_HPP_

#include "motion_utils/motion_utils.hpp"
#include "obstacle_avoidance_planner/common_structs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <chrono>
#include <limits>

enum class LaneChangeDirection
{
  LEFT,
  RIGHT
};
class TrajectoryRemmaper : public rclcpp::Node
{
  static autoware_auto_planning_msgs::msg::Trajectory modifyTrajectory(
    autoware_auto_planning_msgs::msg::Trajectory &input_trajectory,
      LaneChangeDirection lane_change_direction, bool acceleration_profile = false)
  {
    if (input_trajectory.points.empty())
    {
      throw std::invalid_argument("Input trajectory cannot be empty");
    }

    autoware_auto_planning_msgs::msg::Trajectory modified_trajectory = input_trajectory;

    // Lane change dpositionirection multiplier
    double direction_multiplier = (lane_change_direction == LaneChangeDirection::LEFT) ? 1.0 : -1.0;

    // Calculate total trajectory time and length
    double total_time = (modified_trajectory.points.back().pose.position.x - modified_trajectory.points.front().pose.position.x) /
                        modified_trajectory.points.front().longitudinal_velocity_mps;

    double lane_change_length = 3.5; // meters

    // Lateral motion parameters (5th-order polynomial)
    double a0 = 0.0;
    double a1 = 0.0;
    double a2 = 0.0;
    double a3 = 10.0 * lane_change_length / (total_time * total_time * total_time);
    double a4 = -15.0 * lane_change_length / (total_time * total_time * total_time * total_time);
    double a5 =
        6.0 * lane_change_length / (total_time * total_time * total_time * total_time * total_time);

    // Initial conditions
    double initial_velocity = modified_trajectory.points.front().longitudinal_velocity_mps;
    double target_velocity = modified_trajectory.points.back().longitudinal_velocity_mps;
    double velocity_change = target_velocity - initial_velocity;

    double curvature = 0.0;
    // double yaw = 0.0;
    double wheel_base = 2.79;
    double rear_steering_ratio = -0.2;

    for (size_t i = 0; i < modified_trajectory.points.size(); ++i)
    {
      double t = i * (total_time / (modified_trajectory.points.size() - 1.0));

      // Lateral position calculation (5th-order polynomial with direction)
      double lateral_pos = direction_multiplier * (a0 + a1 * t + a2 * t * t + a3 * t * t * t +
                                                   a4 * t * t * t * t + a5 * t * t * t * t * t);

      // Modify y-coordinate for lane change
      modified_trajectory.points[i].pose.position.y += lateral_pos;

      // Velocity and acceleration calculations
      double current_velocity = initial_velocity;
      double current_acceleration = 0.0;

      if (acceleration_profile)
      {
        // Smooth acceleration profile
        if (t <= total_time / 2.0)
        {
          // First half: accelerate
          current_acceleration = 2 * velocity_change / total_time;
          current_velocity = initial_velocity + current_acceleration * t;
        }
        else
        {
          // Second half: maintain velocity
          current_acceleration = 0;
          current_velocity = target_velocity;
        }

        // Update velocity and acceleration in trajectory point
        modified_trajectory.points[i].longitudinal_velocity_mps = current_velocity;
        modified_trajectory.points[i].acceleration_mps2 = current_acceleration;
      }

      // Curvature approximation
      curvature = std::abs(
          (2 * (3 * a3 + 4 * a4 * t + 5 * a5 * t * t)) /
          std::pow(
              1 + std::pow(
                      a1 + 2 * a2 * t + 3 * a3 * t * t + 4 * a4 * t * t * t + 5 * a5 * t * t * t * t, 2),
              1.5));

      // Heading rate calculation
      modified_trajectory.points[i].heading_rate_rps = modified_trajectory.points[i].longitudinal_velocity_mps * curvature;

      // // Yaw calculation for wheel angle
      // yaw = std::atan2(
      //   direction_multiplier *
      //     (a1 + 2 * a2 * t + 3 * a3 * t * t + 4 * a4 * t * t * t + 5 * a5 * t * t * t * t),
      //   modified_trajectory[i].longitudinal_velocity_mps);

      // Wheel angle calculation using wheelbase and rear steering ratio
      modified_trajectory.points[i].front_wheel_angle_rad = std::atan(curvature * wheel_base);
      modified_trajectory.points[i].rear_wheel_angle_rad = rear_steering_ratio * modified_trajectory.points[i].front_wheel_angle_rad;

      // Preserve remaining trajectory information
      modified_trajectory.points[i].pose.position.z = input_trajectory.points[i].pose.position.z;
      modified_trajectory.points[i].pose.orientation = input_trajectory.points[i].pose.orientation;
    }

    return modified_trajectory;
  }

  /**
   * Validate and print trajectory modification details
   */
  static void validateTrajectoryModification(
      const autoware_auto_planning_msgs::msg::Trajectory &original_trajectory,
      const autoware_auto_planning_msgs::msg::Trajectory &modified_trajectory)
  {
    if (original_trajectory.points.size() != modified_trajectory.points.size())
    {
      throw std::runtime_error("Trajectory sizes do not match");
    }

    // Validate key characteristics
    double lateral_change = std::abs(modified_trajectory.points.back().pose.position.y - original_trajectory.points.back().pose.position.y);

    std::cout << "Trajectory Modification Details:" << std::endl;
    std::cout << "Total Points: " << modified_trajectory.points.size() << std::endl;
    std::cout << "Lateral Displacement: " << lateral_change << " meters" << std::endl;
    std::cout << "Initial Position: (" << original_trajectory.points.front().pose.position.x << ", "
              << original_trajectory.points.front().pose.position.y << ")" << std::endl;
    std::cout << "Final Position: (" << modified_trajectory.points.back().pose.position.x << ", "
              << modified_trajectory.points.back().pose.position.y << ")" << std::endl;
  }

public:
  explicit TrajectoryRemmaper(const rclcpp::NodeOptions &node_options);

private:
void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg) const;

rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr traj_pub;
rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr traj_sub;
}

#endif // TRAJECTORY_REMMAPER__NODE_HPP_
