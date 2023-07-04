from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

"""
背景:py02_node_launch.py中 读取yamlNode参数。

需求:在launch文件启动时,动态设置节点中的参数。
1.发起参数( LaunchConfiguration('参数名',);
2.声明参数的值和描述( DeclareLaunchArgument('参数名', default_value='值', description='...') );
3.执行launch文件时,动态传参
"""
def generate_launch_description():
    # 发起的参数对象 = LaunchConfiguration("key")
    para_bg_r = LaunchConfiguration("bg_r")
    para_bg_g = LaunchConfiguration("bg_g")
    para_bg_b = LaunchConfiguration("bg_b")
    
    # 根据"key"值 给 发起的参数对象 赋默认值 和 描述
    # 声明的参数对象 = DeclareLaunchArgument("key", default_value='...', description='red')
    bg_r_cmd = DeclareLaunchArgument(
            'bg_r', default_value='255',
            description='red')

    bg_g_cmd = DeclareLaunchArgument(
            'bg_g', default_value='255',
            description='green')

    bg_b_cmd = DeclareLaunchArgument(
            'bg_b', default_value='255',
            description='blue')

    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node",
        #       节点中的参数(使用declare_parameter): 发起的参数对象
        #       Nav2中的 rewritenyaml可以读取 yaml文件中的指定配置参数 并 形成python对象，可在节点的 parameters中 输入       
        parameters=[{"background_r": para_bg_r},
                    {"background_g": para_bg_g},
                    {"background_b": para_bg_b}]
    )
    
    # 在LaunchDescription中 添加 要使用到的 参数声明的对象 和 启动节点
    return LaunchDescription([bg_r_cmd,bg_g_cmd,bg_b_cmd,turtle])