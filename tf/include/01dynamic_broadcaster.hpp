/*
启动 turtlesim_node节点，转发 乌龟坐标系 相对于 世界坐标系 的位姿。

流程：
    1.包含头文件；
    2.初始化ROS2客户端；
    3.自定义节点类，
     3.1 创建一个动态广播器；
     3.2 创建一个乌龟位姿订阅方；
     3.3 回调函数中，获取乌龟位姿 并生成 相对关系后发布；
    4.调用spin函数，并传入节点对象指针；                 
    5.资源释放。
*/
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class TFDynamicBroadcaster:public rclcpp::Node
{
  public:
    TFDynamicBroadcaster():Node("tf_dynamic_broadcaster_node_cpp")
    {
    //  3.1 创建一个动态广播器,本质是 topic通信，其默认话题是 "/tf"
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //  3.2 创建一个乌龟位姿订阅方
    //  ros2 run turtlesim turtlesim_turtlesim_node
    //  ros2 run turtlesim turtle_teleop_key        使用键盘控制 乌龟 运动。
    pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10,
      std::bind(&TFDynamicBroadcaster::do_pose, this, std::placeholders::_1)); 
    }
    
 private:
    //  3.3 回调函数中，获取乌龟位姿 并生成 相对关系后发布；
    void do_pose(const std::shared_ptr<turtlesim::msg::Pose> msg)
    // "/turtle1/pose"话题发布的数据 是 std::shared_ptr<turtlesim::msg::Pose> 类型
    //                            不是 turtlesim::msg::Pose& pose
    {
        // 组织消息
        geometry_msgs::msg::TransformStamped transform;
        
        transform.header.stamp = this->now();
        transform.header.frame_id = "world";

        transform.child_frame_id = "turtle1";
        
        // 平移量
        transform.transform.translation.x = msg->x;
        transform.transform.translation.y = msg->y;
        transform.transform.translation.z = 0.0;
        
        // 从欧拉角转化为四元数
         // 只有yaw的值， roll,pitch都是0
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, msg->theta);

        transform.transform.rotation.x = qtn.x();
        transform.transform.rotation.y = qtn.y();
        transform.transform.rotation.z = qtn.z();
        transform.transform.rotation.w = qtn.w();

        // 发布后， 使用rviz2可视化
        broadcaster_->sendTransform(transform);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};