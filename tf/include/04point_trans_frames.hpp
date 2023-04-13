/*
需求： 实现坐标点 在 不同坐标系下的变换

已知：
  1.坐标系相对关系(laser->base_link);
  2.点在 laser坐标系 的位置。

求解：点 在base_link的坐标。 

实现：
  step1.使用 坐标系变换监听方 订阅 laser->base_link 坐标(系)变换 存入 buffer;
        还需 创建一个专门的 订阅方 订阅坐标点数据。

  step2.求解，
        创建一个过滤器对象， 输入 tf树 和 坐标点。
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// 监听方 需要的头文件
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// 
#include "tf2_ros/create_timer_ros.h"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "message_filters/subscriber.h"
#include "tf2_ros/message_filter.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;

class PointTransFrames:public rclcpp::Node{
  public:
     PointTransFrames():Node("point_transform_frames"){
       // 坐标系转换 监听方
       buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
       timer_interface_ = std::make_shared<tf2_ros::CreateTimerROS>(
                  this->get_node_base_interface(), 
                  this->get_node_timers_interface());

       buffer_->setCreateTimerInterface(timer_interface_);

       listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
       
       // 坐标点的消息订阅
       point_sub_.subscribe(this, "point");
       
       // 将 laser->base_link的坐标系转换 和 laser坐标系中的坐标点 融合
       // 创建 过滤器
       // 订阅对象， 坐标监听缓存，父坐标系， 队列size
       filter_ = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>>(
          point_sub_,
          *buffer_,
          "base_link",
          10,
          this->get_node_logging_interface(),
          this->get_node_clock_interface(),
          tf2::durationFromSec(0.2) // timeout duration after requesting transforms from buffer
                                    // 0.2s内要查询到 buffer中 坐标系间的转换
       );

       // 坐标点转换
       filter_->registerCallback(&PointTransFrames::transform_point, this);
     }
  
  private:
    void transform_point(std::shared_ptr<geometry_msgs::msg::PointStamped> ps){
      // 实现 base_link下的坐标点 转化到 laser坐标系
      geometry_msgs::msg::PointStamped transformed_point = buffer_->transform(*ps, "base_link");
      RCLCPP_INFO(this->get_logger(),"父级坐标系：%s, 坐标:(%.2f, %.2f, %.2f)",
                  transformed_point.header.frame_id.c_str(),
                  transformed_point.point.x,
                  transformed_point.point.y,
                  transformed_point.point.z
                 );
    }
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    std::shared_ptr<tf2_ros::CreateTimerROS> timer_interface_;

    message_filters::Subscriber<geometry_msgs::msg::PointStamped> point_sub_;

    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PointStamped>> filter_;
};


// 续：在ROS2的 TF 框架中 除了封装了 坐标系广播与订阅 功能外， 还提供一些工具，涉及到两个功能包：tf2_ros、tf2_tools。
// tf2_ros功能包中 提供的常用 节点 如下：
// · static_transform_publisher，该节点用于 广播静态坐标变换；
// · tf2_monitor, 该节点用于打印 所有 或 特定坐标系间的发布频率 与 网络延迟；
// · tf2_echo, 该节点用于打印 特定坐标系间的平移旋转关系。