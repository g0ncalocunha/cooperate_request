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

// #include "lane_change_request/lane.hpp"
#include "rclcpp/time.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

#include "rclcpp/rclcpp.hpp"
// #include "vehicle_info_util/vehicle_info_util.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <chrono>
#include <limits>

bool lane_change_direction = true;
bool enable_lane_change = false;
bool acceleration_profile = false;

namespace trajectory_remap_node
{
class TrajectoryRemapper : public rclcpp::Node
{
  public:
   TrajectoryRemapper(const rclcpp::NodeOptions &node_options)
      : Node("trajectory_remapper", node_options)
  {
    // interface publisher
    traj_pub = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/path", 1);
  
    // interface subscriber
    traj_sub = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
        "~/output/remmaped_path", 1, std::bind(&TrajectoryRemapper::trajectoryCallback, this, std::placeholders::_1));
  }
  static autoware_auto_planning_msgs::msg::Trajectory modifyTrajectory(
    autoware_auto_planning_msgs::msg::Trajectory &input_trajectory,
      bool lane_change_direction = true, bool acceleration_profile = false)
  {
    if (input_trajectory.points.empty())
    {
      throw std::invalid_argument("Input trajectory cannot be empty");
    }

    autoware_auto_planning_msgs::msg::Trajectory modified_trajectory = input_trajectory;

    // Lane change dpositionirection multiplier
    double direction_multiplier = (lane_change_direction == true) ? 1.0 : -1.0;

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

void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  autoware_auto_planning_msgs::msg::Trajectory new_trajectory = *msg;
  autoware_auto_planning_msgs::msg::Trajectory modified_trajectory = autoware_auto_planning_msgs::msg::Trajectory();

  if (!enable_lane_change)
  {
    traj_pub->publish(new_trajectory);
    return;
  }
  else
  {
    // Direct cmd
    modified_trajectory = TrajectoryRemapper::modifyTrajectory(new_trajectory,lane_change_direction, acceleration_profile);
    traj_pub->publish(modified_trajectory);
  }
}
rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr traj_pub;
rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr traj_sub;
};

} // namespace trajectory_remap_node

int main(int argc, char *argv[])
{

  std::cout << "Starting Trajectory-Remapper..." << std::endl;

  // data_mqtt_server data_mqtt = readMqttData();
  // spdlog::debug("MQTT address: {}", data_mqtt.address);
  // spdlog::debug("MQTT username: {}", data_mqtt.user_name);
  // spdlog::debug("MQTT password: {}", data_mqtt.password);

  // MqttWrapper *mqtt_server;

  // if (mqtt_enable)
  // {
  //   mqtt_server = new MqttWrapper(data_mqtt, on_message_mqtt);
  //   // Wait until connect
  //   while (!mqtt_server->is_connected())
  //   {
  //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //     spdlog::info("Connected to MQTT server");
  //   }
  // }
  // else
  // {
  //   spdlog::info("MQTT is disabled");
  // }

  // spdlog::info("Connected to MQTT server");

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto trajectory_remapper = std::make_shared<trajectory_remap_node::TrajectoryRemapper>(options);
  rclcpp::spin(trajectory_remapper);
  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(node);
  // std::thread executor_thread([&executor]()
  //                             { executor.spin(); });

  // // DDS
  // std::cout << "Setting up DDS..." << std::endl;
  // dds_ = new Dds("DenmNode", domain_id, on_message_dds);
  // dds_->subscribe("aw/in/cmd/steering_tire_angle");
  // dds_->subscribe("aw/in/cmd/acceleration");
  // dds_->subscribe("aw/in/cmd/speed");
  // dds_->subscribe("aw/in/cmd/hazard_lights");
  // std::cout << "DDS set up" << std::endl;

  // while (rclcpp::ok())
  // {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
  // }

  // // If rclcpp::ok() returns false, it means ros2 has been shutdown,
  // // so join the executor thread before closing the application
  // if (executor_thread.joinable())
  // {
  //   executor_thread.join();
  // }

  rclcpp::shutdown();

  return 0;
}
