import os

from launch import LaunchDescription
from launch_ros.actions import Node

# 声明参数与获取 配套使用
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

# 在launch文件启动时,导入yaml文件中的参数配置，动态修改Node中的参数值。
def generate_launch_description():
    
    parameter_node = Node(
        # 包名
        package="parameter",
        # 可执行文件
        executable="parameter_node",
        # 待launch的节点名称
        name="parameter_node",

        # 方式1：通过 键-值 对的方式对参数进行赋值
        # parameters=[{"my_parameter":"xxx"}]
        # 方式2：通过yaml路径的动态读取获得参数(install目录下的)
        parameters=[os.path.join(get_package_share_directory("parameter"),"params","params.yaml")]
    )

    return LaunchDescription([parameter_node])



# 在launch文件启动时,动态修改Node中的参数值。
# 实现步骤：
#         1.声明参数( DeclareLaunchArgument(键值对...) );
#         2.调用参数( LaunchConfiguration()通过键 来 获得值);
#         3.执行launch文件时,动态导入


# def generate_launch_description():
#     # 1.声明参数                           key:my_parameter           value:earth
#     #                                   (与Node中的参数名相同)        
#     parameter = DeclareLaunchArgument(name = "my_parameter", default_value = "sss")

#     # 2.调用参数
#     parameter_node = Node(
#        package = "parameter",
#        executable = "parameter_node",
#        parameters = [
#         {"my_parameter": LaunchConfiguration("my_parameter")}
#         ] 
#     # parameters = [
#     #   {"与Node中的参数名相同":LaunchConfiguration("可执行文件名称")}
#     #   ]
#     )
#     return LaunchDescription([parameter, parameter_node])
