cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate custom services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanMotion.srv"
  DEPENDENCIES std_msgs
)

# Add custom service dependencies to executables
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Install directories
install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/python/collision_publisher.py
  scripts/python/data_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

# Create motion planning service
add_executable(motion_planning_service scripts/cpp/motion_planning_service.cpp)
ament_target_dependencies(motion_planning_service 
  rclcpp 
  moveit_ros_planning_interface
  std_srvs
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(motion_planning_service "${cpp_typesupport_target}")

# Install executables
install(TARGETS 
  motion_planning_service
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
