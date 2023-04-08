"""
   需求:为turtlesim_node绑定事件,节点启动时,执行生成新的乌龟程序
       节点关闭时，执行日志输出操作
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess

# 注册事件
from launch.actions import RegisterEventHandler, LogInfo

# 执行事件
from launch.event_handlers import OnProcessStart, OnProcessExit


def generate_launch_description():
    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    # ros2 service call /spawn turtlesim/srv/Spawn "{'x': 8.0, 'y': 3.0}"
    # shell命令对象
    spawn = ExecuteProcess(
       cmd = ["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': 8.0, 'y': 3.0}\""],
       output="both",
       shell=True
    )
    
    # 注册事件1
    event_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=turtle,
            on_start=spawn
        )
    )
    
    # 注册事件2
    event_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=turtle,
            on_exit=[LogInfo(msg="turtlesim_node退出")]
        )
    )

    # # 注册事件3:满足某个条件执行事件IfCondition(...)
    # event_exit = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=turtle,
    #         on_exit=[LogInfo(msg="turtlesim_node退出")]
    #     )
    # )

    return LaunchDescription([turtle,event_start,event_exit])