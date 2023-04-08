# 节点设置：launch中需要执行的节点 被 封装为 launch_ros.actions import Node对象

# 执行指令：launch中需要执行的命令 被 封装为 launch.actions.ExcuteProcess对象

# 参数设置：参数设置主要涉及到参数的声明与调用两个部分，
#         声明被封装为：launch.actions.DeclareLaunchArgment;
#         调用被封装为：launch.substitutions import LaunchConfiguration

# 文件包含：launch文件中可以包含其他的launch文件，需要使用的API为 
#         launch.actions.IncludeLaunchDescription 和 launch.launch_description_sources.PythonLaunchDescriptionSource

# 分组设置：launch文件中，为了方便管理可以对节点分组，需要使用的API为 

# 添加事件：节点在运行过程中在某个时机可以触发不同的事件，可对事件注册回调函数。
#         需要使用的API为
#          launch.actions.RegisterEventHandler、launch.event_handlers.OnProcessStart
#          launch.event_handlers.OnProcessExit

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    t1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="t1")

    t2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="t2")

    return LaunchDescription([t1,t2])