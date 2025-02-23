cmake_minimum_required(VERSION 3.8)
project(ur5_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# # Install urdf/xacro files
install(DIRECTORY launch meshes config urdf rviz
  DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY urdf/
#   DESTINATION share/${PROJECT_NAME}/urdf
#   FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
# )

# # Install RViz configuration
# install(DIRECTORY rviz/
#   DESTINATION share/${PROJECT_NAME}/rviz
# )

# # Install launch files
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch
# )

# # Install mesh files
# install(DIRECTORY meshes/
#   DESTINATION share/${PROJECT_NAME}/meshes
#   FILES_MATCHING PATTERN "*.dae" PATTERN "*.stl"
# )

# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
#   FILES_MATCHING PATTERN "*.yaml"
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
