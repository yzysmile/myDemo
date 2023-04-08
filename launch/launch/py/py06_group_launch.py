"""
需求：创建3个turtlesim_node，前两个为一个group。

"""

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    t1 = Node(package="turtlesim",executable="turtlesim_node",name="t1")
    t2 = Node(package="turtlesim",executable="turtlesim_node",name="t2")
    t3 = Node(package="turtlesim",executable="turtlesim_node",name="t3")
    # 分组
    # 设置当前组命令空间
    g1 = GroupAction(actions=[PushRosNamespace("g1"),t1,t2])
    g2 = GroupAction(actions=[PushRosNamespace("g2"),t3])
    return LaunchDescription([g1,g2])