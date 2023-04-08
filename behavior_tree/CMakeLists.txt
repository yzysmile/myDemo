cmake_minimum_required(VERSION 3.8)
project(behavior_tree)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_executable(first_behavior_tree src/behavior_tree/00your_first_behavior_tree.cpp)
ament_target_dependencies(first_behavior_tree rclcpp behaviortree_cpp_v3)
install(TARGETS 
        first_behavior_tree
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(blackboard_and_ports src/behavior_tree/01blackboard_and_ports.cpp)
ament_target_dependencies(blackboard_and_ports rclcpp behaviortree_cpp_v3)
install(TARGETS 
        blackboard_and_ports
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(ports_with_generic_types src/behavior_tree/02ports_with_generic_types.cpp)
ament_target_dependencies(ports_with_generic_types rclcpp behaviortree_cpp_v3)
install(TARGETS 
        ports_with_generic_types
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(reactive_behavior src/behavior_tree/03reactive_behavior.cpp)
ament_target_dependencies(reactive_behavior rclcpp behaviortree_cpp_v3)
install(TARGETS 
        reactive_behavior
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(subtrees src/behavior_tree/04subtrees.cpp)
ament_target_dependencies(subtrees rclcpp behaviortree_cpp_v3)
install(TARGETS 
        subtrees
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(port_remapping src/behavior_tree/05port_remapping.cpp)
ament_target_dependencies(port_remapping rclcpp behaviortree_cpp_v3)
install(TARGETS 
        port_remapping
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
