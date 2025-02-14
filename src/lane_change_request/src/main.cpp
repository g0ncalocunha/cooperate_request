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

#include "lane_change_request/lane.hpp"
#include "rclcpp/time.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

static LaneChangeDirection lane_change_direction = LaneChangeDirection::LEFT;
bool enable_lane_change = false;
bool acceleration_profile = false;

TrajectoryRemmaper::TrajectoryRemmaper(const rclcpp::NodeOptions &node_options)
    : Node("trajectory_remmaper", node_options)
{
  // interface publisher
  traj_pub = create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/path", 1);

  // interface subscriber
  traj_sub = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "~/output/remmaped_path", 1, std::bind(&TrajectoryRemmaper::trajectoryCallback, this, std::placeholders::_1));
}
// void TrajectoryRemmaper::onPath(const Path::SharedPtr path_ptr)
// {
//   time_keeper_ptr_->init();
//   time_keeper_ptr_->tic(__func__);

//   // 0. return if path is backward
//   // TODO(murooka): support backward path
//   const auto is_driving_forward = driving_direction_checker_.isDrivingForward(path_ptr->points);
//   if (!is_driving_forward)
//   {
//     RCLCPP_WARN_THROTTLE(
//         get_logger(), *get_clock(), 5000,
//         "Backward path is NOT supported. Just converting path to trajectory");

//     const auto traj_points = trajectory_utils::convertToTrajectoryPoints(path_ptr->points);
//     const auto output_traj_msg = trajectory_utils::createTrajectory(path_ptr->header, traj_points);
//     traj_pub_->publish(output_traj_msg);
//     return;
//   }

//   // 1. create planner data
//   const auto planner_data = createPlannerData(*path_ptr);

//   // 2. generate optimized trajectory
//   const auto optimized_traj_points = generateOptimizedTrajectory(planner_data);

//   // 3. extend trajectory to connect the optimized trajectory and the following path smoothly
//   auto full_traj_points = extendTrajectory(planner_data.traj_points, optimized_traj_points);

//   // 3.a) set lane change parameters
//   LaneChangeDirection lane_change_direction_ = LaneChangeDirection::LEFT;
//   bool enable_lane_change_ = true;
//   bool enable_acceleration_ = false;

//   // 3.b) if enable_lane_change_ is true, change trajectory

//   if (enable_lane_change_)
//   {
//     full_traj_points = modifyTrajectory(full_traj_points, lane_change_direction_, enable_acceleration_);
//   }

//   time_keeper_ptr_->toc(__func__, "");
//   *time_keeper_ptr_ << "========================================";
//   time_keeper_ptr_->std::endline();

//   // publish calculation_time
//   // NOTE: This function must be called after measuring onPath calculation time
//   const auto calculation_time_msg = createStringStamped(now(), time_keeper_ptr_->getLog());
//   debug_calculation_time_pub_->publish(calculation_time_msg);

//   const auto output_traj_msg =
//       trajectory_utils::createTrajectory(path_ptr->header, full_traj_points);
//   traj_pub_->publish(output_traj_msg);
// }

void TrajectoryRemmaper::trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr &msg) const
{
  autoware_auto_planning_msgs::msg::Trajectory new_trajectory = msg;
  autoware_auto_planning_msgs::msg::Trajectory modified_trajectory = autoware_auto_planning_msgs::msg::Trajectory();

  if (!enable_lane_change)
  {
    traj_pub->publish(new_trajectory);
    return;
  }
  else
  {
    // Direct cmd
    modified_trajectory = TrajectoryRemmaper::modifyTrajectory(new_trajectory,lane_change_direction, acceleration_profile);
    traj_pub->publish(modified_trajectory);
  }
}

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
  auto trajectory_remmaper = std::make_shared<TrajectoryRemmaper>(options);
  rclcpp::spin(trajectory_remmaper);
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
