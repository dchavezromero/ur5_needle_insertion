cmake_minimum_required(VERSION 3.22)
project(ur5_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()


add_executable(move_ur5 cpp/move_ur5.cpp)
ament_target_dependencies(move_ur5 rclcpp moveit_ros_planning_interface)

install(TARGETS move_ur5
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
