#include <rclcpp/rclcpp.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>
#include <tier4_rtc_msgs/msg/cooperate_command.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <random>

using CooperateCommands = tier4_rtc_msgs::srv::CooperateCommands;
using UUID = unique_identifier_msgs::msg::UUID;

unique_identifier_msgs::msg::UUID generateUUID() {
  unique_identifier_msgs::msg::UUID uuid_msg;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, 255);

  for (int i = 0; i < 16; i++) {
    uuid_msg.uuid[i] = static_cast<uint8_t>(dis(gen));
  }

  return uuid_msg;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (argc != 1) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add direction of lane change request");
      return 1;
  }
  // Create a client to call the service
  std::shared_ptr<rclcpp::Node>  node = rclcpp::Node::make_shared("lane_change_request_node");
  rclcpp::Client<tier4_rtc_msgs::srv::CooperateCommands>::SharedPtr client = 
    node->create_client<tier4_rtc_msgs::srv::CooperateCommands>("/planning/cooperate_commands/lane_change_right");

  // Wait for the service to be available
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service not available");
    return 1;
  }

  // Create the request
  auto request = std::make_shared<CooperateCommands::Request>();
  auto command = tier4_rtc_msgs::msg::CooperateCommand();
  UUID uuid = generateUUID();  // Generate a valid UUID
  command.uuid = uuid;
  command.module.type = 16;
  command.command.type = 0;
  request->commands.push_back(command);

  // while (client->wait_for_service(std::chrono::seconds(3))) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
  //     return 0;
  //   }
  // }


  // Send the request
  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lane change request sent successfully");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to send lane change request");
  }
  
  rclcpp::shutdown();
  return 0;
}