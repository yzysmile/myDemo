/*
  坐标变换监听： 
      1.实现 坐标点 在不同坐标系之间的变换；
      2.实现 不同坐标系 之间的变换。
     （前提是 广播不同坐标系 关系，静态广播 或 动态广播均可）

  需求：
     发布 laser到 base_link的 坐标变换、camera到base_link的坐标系相对关系，求解 laser(子坐标系)到camera(父坐标系)的相对关系。

     具体步骤为：step1.创建一个缓存对象(在构造此对象时 要传入clock)，可以融合多个 坐标系间的相对位置关系 形成一颗坐标树；

               step2.创建一个监听器，绑定缓存对象，将所有广播器广播的数据写入缓存；
                     (监听话题 "/tf_static"、"/tf")

               step3.编写定时器，循环实现转换。 
*/
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

class TFListener:public rclcpp::Node
{
  public:
    TFListener():Node("tf_listener_demo")
    { 
      buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      
      // 订阅到的 坐标(系)相对转化关系 存入buffer
      listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
      
      timer_ = this->create_wall_timer(1s, std::bind(&TFListener::on_timer, this));
    }

  private:
    void on_timer(){
      // 当坐标(系)转化失败(e.g buffer_中 不存在 camera、laser的坐标系)时，会抛出异常，使用 try-catch机制
       // 或使用 bool canTransform(const std::string& target_frame, const std::string& source_frame,
       //                           const TimePoint& time, std::string* error_msg = nullptr) const override
       // 提前进行是否能进行 坐标系间的 转换
      try{
        // 实现坐标系转换,TimePoint表示查询具体时间帧的转换(子坐标系-target_frame 转化到 父坐标系-source_frame 的转化关系)
        // lookupTransform(const std::string& target_frame, 
        //                 const std::string& source_frame
        //                 const tf2::TimePoint& time)
        // tf2::TimerPointZero, 转换最新时刻的坐标帧
       geometry_msgs::msg::TransformStamped ts = buffer_->lookupTransform("camera", "laser", tf2::TimePointZero);
         RCLCPP_INFO(this->get_logger(),"---------转换完成的坐标帧信息-------");
         RCLCPP_INFO(this->get_logger(),"父坐标系：%s, 子坐标系：%s, 偏移量(%.2f, %.2f, %.2f)",
                                        ts.header.frame_id.c_str(), ts.child_frame_id.c_str(), 
                                        ts.transform.translation.x,
                                        ts.transform.translation.y,
                                        ts.transform.translation.z);
      }catch(const tf2::TransformException & ex){
        RCLCPP_INFO(this->get_logger(),"异常提示:%s", ex.what());
      }
    }
    
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_{nullptr};
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
};