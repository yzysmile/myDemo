from launch import LaunchDescription
from launch_ros.actions import Node

# 声明参数与获取 配套使用
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

# 在launch文件启动时,动态修改Node中的参数值。
# 实现步骤：
#         1.声明参数( DeclareLaunchArgument(键值对...) );
#         2.调用参数( LaunchConfiguration()通过键 来 获得值);
#         3.执行launch文件时,动态导入


def generate_launch_description():
    # 1.声明参数                           key:my_parameter           value:earth
    #                                   (与Node中的参数名相同)        
    parameter = DeclareLaunchArgument(name = "my_parameter", default_value = "sss")
    
    # parameter_yaml = DeclareLaunchArgument('params_file',default_value = os.path.join(bringup, 'params', 'params.yaml'))


    # 2.调用参数
    parameter_node = Node(
       package = "parameter",
       executable = "parameter_node",
       parameters = [
        {"my_parameter": LaunchConfiguration("my_parameter")}
        ] 
    # [
    # {"与Node中的参数名相同":LaunchConfiguration("可执行文件名称")}
    # ]
    )
    return LaunchDescription([parameter, parameter_node])
