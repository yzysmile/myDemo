#include "../include/00static_broadcaster.hpp"


// int main(int argc,char* argv[]) 也可以写成 int main(int argc,char** argv)；
// argc表示程序运行时 发送给main函数的 命令行参数的个数；
// 第0个参数是 可执行程序，第1~n个参数 是 命令行传入的参数，故 argc = 命令行传入的参数个数+1;
// argv[]是字符指针数组，它的每个元素都是字符指针，指向命令行中每个参数的第一个字符。
int main(int argc, char* argv[])
{
  // 判断传入的参数是否合法
  if(argc != 9){
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "传入的参数不合法!");
    return 1;
  }
  
  // 初始化
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<TFStaticBroadcaster>(argv));
  rclcpp::shutdown();
  return 0;
}