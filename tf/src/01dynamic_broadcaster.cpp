#include "../include/01dynamic_broadcaster.hpp"

int main(int argc ,char* argv[])
{
  // 初始化
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<TFDynamicBroadcaster>());

  //释放资源
  rclcpp::shutdown();
  return 0;
}