"""
    需求:在当前launch文件中,包含其他launch文件
"""

from launch import LaunchDescription
from launch_ros.actions import Node
# 文件包含相关
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# 获取功能包下share目录路径
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    include = IncludeLaunchDescription(
        #
       launch_description_source=PythonLaunchDescriptionSource(
                                               # share下的launch文件夹
           launch_file_path=os.path.join(get_package_share_directory("launch"),
                                         "launch/py",
                                         "py04_args_launch.py"
            )
        ),
        # 包含的launch文件中 声明(DeclareLaunchArgument) 3个参数
        launch_arguments=[("bg_r","80"),("bg_g","80"),("bg_b","80")]
    )
    
    return LaunchDescription([include])