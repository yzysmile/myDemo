/*
    tf(transform frame)坐标变换
    构成：广播方 + 监听方；
         广播方：发布 两个坐标系（右手坐标系） 之间的转换, 本质是 topic 通信；
         监听方：订阅 多个广播方 发布的数据(多对一，是 使用tf2_ros::Buffer的原因)，多个 坐标系之间的转换关系 将自动 形成 tf树；
                tf树 中的 可实现树中任意两个坐标系的转换；
                tf树 中的 箭头从 父坐标系 指向 子坐标系。

    值得注意的是：
        坐标变换时，需要参考消息数据中的时间戳；
        需保证两个坐标帧在一定的时间差范围内。

    接口消息：
        geometry_msgs/msg/transform_stamped.hpp(时间 父坐标系名称 子坐标系名称 平移 旋转)
        #include "geometry_msgs/msg/transform_stamped.hpp"

        geometry_msgs/msg/PointStamped(时间 坐标系名称 点坐标)
        #include "geometry_msgs/msg/point_stamped.hpp"

    坐标系相对关系：
        静态坐标系相对关系——两个坐标系的关系是固定不变的，
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_
        broadcaster_->sendTransform(transform),只需发布一次。

        动态坐标系相对关系——两个坐标系的关系是动态变化的。
        std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_
        broadcaster_->sendTransform(transform)，需重复发布。

    核心：
        如何 广播 两个坐标系的相对关系（静态广播 在tf2中有内置相关工具，可直接执行节点并传入 表示坐标系间 相对关系的参数),
        ros2 run tf2_ros static_transform_publish 0.1 0 0.2 0 0 0 base_link    laser
        //                                          平移     旋转    父坐标系    子坐标系
        //                                        (x y z)(yaw pitch roll)

        如何使用rviz2显示坐标系相对关系。
        // 指定 固定坐标系(通常为 父坐标系)；
        // 红色轴为x轴，绿色轴为y轴，蓝色轴为z轴(右手坐标系)；
        // rviz2中 父坐标系 指向 子坐标系

    依赖：rclcpp tf2_ros(广播类 监听类) geometry(消息接口) tf2(欧拉角与四元数 转换类)

         #include "rclcpp/rclcpp.hpp"
         
         广播类
         #include "tf2_ros/static_transform_broadcaster.h"
         #include "tf2_ros/transform_broadcaster.h"
         
         监听类
         #include "tf2_ros/transform_listener.h"
         #include "tf2_ros/buffer.h"

         消息接口类
         #include "geometry_msgs/msg/transform_stamped.hpp"
         #include "geometry_msgs/msg/point_stamped.hpp"
         
         欧拉角与四元数 转换类
         #include "tf2/LinearMath/Quaternion.h"

         ros2 pkg create TF --build-type ament_cmake --dependencies rclcpp tf2 tf2_ros geometry_msgs turtlesim
*/
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class TFStaticBroadcaster:public rclcpp::Node
{
  public:
    TFStaticBroadcaster(char* argv[]):Node("tf_static_broadcaster_node_cpp"){
       
       // 广播对象实例化，本质是 topic通信， 默认话题是 "/tf_static"
       broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
       
       // 组织并发布数据
       pub_static_tf(argv);
    }

  private:
     // 广播对象
     std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
     
     void pub_static_tf(char * argv[]){
        // 消息
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now(); // 时间戳
        transform.header.frame_id = argv[7]; // 父坐标系

        transform.child_frame_id = argv[8]; // 子级坐标系
        
        // 平移量                            atof:字符串 转化为 浮点数
        transform.transform.translation.x = atof(argv[1]);
        transform.transform.translation.y = atof(argv[2]);
        transform.transform.translation.z = atof(argv[3]);

        // 设置四元数
         // 将欧拉角转换为四元数
        tf2::Quaternion qtn;
        qtn.setRPY(atof(argv[4]), atof(argv[5]), atof(argv[6]));
        transform.transform.rotation.x = qtn.x();
        transform.transform.rotation.y = qtn.y();
        transform.transform.rotation.z = qtn.z();
        transform.transform.rotation.w = qtn.w();

        // 发布,本质是 topic通信
        broadcaster_->sendTransform(transform);
     }
};




