#include "rclcpp/rclcpp.hpp"
#include "control/gripper_action_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true); // Enable intra-process communication

  auto node = std::make_shared<GripperActionServer>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
