#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Create a node with the name "cpp_node"
  auto node = rclcpp::Node::make_shared("cpp_node");

  // Log a message to indicate that the node has started
  RCLCPP_INFO(node->get_logger(), "CPP Node has been started successfully!");

  // Spin the node to keep it alive
  rclcpp::spin(node);

  // Shutdown the ROS 2 system
  rclcpp::shutdown();
  
  return 0;
}