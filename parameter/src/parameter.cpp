# include "../include/parameter.hpp"

void ParameterDemo::ParameterClass::respond()
{
  /* 获取"my_parameter"参数的值 并赋给 成员变量"parameter_string_" */
  this->get_parameter("my_parameter", parameter_string_);
  RCLCPP_INFO(this->get_logger(),"Hello %s", parameter_string_.c_str());

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParameterDemo::ParameterClass>());
  rclcpp::shutdown();
  return 0;
}