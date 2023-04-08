"""
Node的使用,几个传参时的重要参数。
Node(
      package         所属的功能包
      executable      可执行程序
      name            节点名称
      namespace       节点的命令空间
      parameter       设置参数
      remapping       话题重映射
      arguments       为节点传参["--ros-args","xx","yy","zz"]
    )

    重映射：
    1.把一个node原本发布的topic,映射为另外一个名字;
    2.把其他node发布的原始的topic,映射为所需要的topic。

"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # 一个Node中必须指定两个参数： package、executable
    # "turtle1" 表示Node的一个对象
    turtle1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        arguments=["--ros-args","--remap","__ns:=/t2"] 
    )
    
    turtle2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="demo_Node",
        # 节点自动保持重启状态
        respawn=True,
        # （当前节点运行中）
        # 使用 ros2 param dump <node_name> --output-dir <保存路径> 可自动生成yaml文件
         # e.g 本例中：ros2 param dump demo_Node --output-dir src/launch/config
        parameters=[{"background_r": 255, "background_g": 255, "background_b": 0}]
    )

    turtle3 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="demo_Node2",
        

        # 获取install路径下的yaml文件参数
        parameters=[os.path.join(get_package_share_directory("launch"),"config","demo_Node.yaml")]
    )
    # [] 表示一个list
    return LaunchDescription([turtle1, turtle2, turtle3])