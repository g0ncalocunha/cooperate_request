#include <rclcpp/rclcpp.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <random>

using CooperateCommands = tier4_rtc_msgs::srv::CooperateCommands;
using UUID = unique_identifier_msgs::msg::UUID;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("lane_change_request_node");

  // Create a client to call the service
  auto client = node->create_client<CooperateCommands>("/cooperate_commands");

  // Wait for the service to be available
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Service not available");
    return 1;
  }

  // Create the request
  auto request = std::make_shared<CooperateCommands::Request>();

  // Set a valid UUID for the request
  UUID uuid;
  // Generate a valid UUID (pseudo example)
  uuid.uuid = generateUUID  // Replace with actual generation code
  autoware_auto_msgs::msg::CooperateCommand command;
  command.uuid = uuid;
  command.module = 15;
  command.direction = 0;  // Force lane change to the right

  request->commands.push_back(command);

  // Send the request
  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Lane change request sent successfully");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to send lane change request");
  }

  rclcpp::shutdown();
  return 0;
}

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
