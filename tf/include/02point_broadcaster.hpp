/*
   需求： 发布 laser坐标系下的 坐标点 数据
*/
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace std::chrono_literals;

class PointBroadcaster:public rclcpp::Node
{
  public:
    PointBroadcaster():Node("point_broadcaster"),x_(0.0)
    {
      point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point", 10);

      timer_ = this->create_wall_timer(1s,std::bind(&PointBroadcaster::on_timer, this));
    }
  
  private:
    void on_timer()
    {
        // 组织消息
        geometry_msgs::msg::PointStamped ps;
        ps.header.stamp = this->now();
        ps.header.frame_id = "laser";
        x_ += 0.05;

        ps.point.x = x_;
        ps.point.y = 0.0;
        ps.point.z = -0.1;

        // 发布消息
        point_pub_->publish(ps);

        RCLCPP_INFO(this->get_logger(),"坐标系：%s, 坐标:(%.2f, %.2f, %.2f)",
                    ps.header.frame_id.c_str(),
                    ps.point.x,
                    ps.point.y,
                    ps.point.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    double x_;
};