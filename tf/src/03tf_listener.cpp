#include "../include/03tf_listener.hpp"

int main(int argc ,char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}