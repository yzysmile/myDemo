#include <chrono>
#include <string>
#include <functional>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/*
ROS2可以在一个Node声明参数(parameter)
   其使用核心是：declare_parameter<Type>("参数名"，该参数名字的默认值)；
                get_parameter("参数名"， 参数名对应的参数值要赋给的变量)

   可通过launch、yaml文件组合使用 配置 “参数名” 对应的 “参数值”，本demo通过此方法配置参数值；

   也可通过终端执行命令， 修改“参数名” 对应的 “参数值”，e.g
    ros2 param set /parameter_node my_parameter positive
*/

namespace ParameterDemo{

class ParameterClass:public rclcpp::Node
{
public:
    ParameterClass():Node("parameter_node"){
      /* 声明节点中有一个类型为"string"，名为"my_parameter"的参数，默认值为"world". */
      declare_parameter<std::string> ("my_parameter", "yzy");

      timer_ = this->create_wall_timer(1000ms,std::bind(&ParameterClass::respond, this));
    }
    
    void respond();

private:
  std::string parameter_string_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  //  namespace ParameterDemo