cmake_minimum_required(VERSION 3.8)
project(tf)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)


add_executable(static_broadcaster src/00static_broadcaster.cpp)
ament_target_dependencies(static_broadcaster rclcpp tf2 tf2_ros geometry_msgs)
install(TARGETS 
        static_broadcaster
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(dynamic_broadcaster src/01dynamic_broadcaster.cpp)
ament_target_dependencies(dynamic_broadcaster rclcpp tf2 tf2_ros geometry_msgs tf2_geometry_msgs turtlesim)
install(TARGETS 
        dynamic_broadcaster
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(point_broadcaster src/02point_broadcaster.cpp)
ament_target_dependencies(point_broadcaster rclcpp tf2_ros geometry_msgs)
install(TARGETS 
        point_broadcaster
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(tf_listener src/03tf_listener.cpp)
ament_target_dependencies(tf_listener rclcpp tf2_ros tf2 geometry_msgs)
install(TARGETS 
        tf_listener
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(point_trans_frames src/04point_trans_frames.cpp)
ament_target_dependencies(point_trans_frames rclcpp tf2_ros tf2 geometry_msgs tf2_geometry_msgs message_filters)
install(TARGETS 
        point_trans_frames
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
