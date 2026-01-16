cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch config rviz maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
