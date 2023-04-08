from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

"""
背景:py02_node_launch.py中 读取yamlNode参数。

需求:在launch文件启动时,动态设置节点中的参数。
1.声明参数( DeclareLaunchArgument(键值对...) );
2.调用参数( LaunchConfiguration()通过键 来 获得值);
3.执行launch文件时,动态传参
"""
def generate_launch_description():
    # 参数对象                        # 键            值
    bg_r = DeclareLaunchArgument(name="bg_r", default_value="255")
    bg_g = DeclareLaunchArgument(name="bg_g", default_value="255")
    bg_b = DeclareLaunchArgument(name="bg_b", default_value="25")

    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
        #              节点中的参数    LaunchConfiguration（键）即为值
        parameters=[{"background_r": LaunchConfiguration("bg_r"),
                     "background_g": LaunchConfiguration("bg_g"),
                     "background_b": LaunchConfiguration("bg_b"),
                     }]
    )

    return LaunchDescription([bg_r,bg_g,bg_b,turtle])