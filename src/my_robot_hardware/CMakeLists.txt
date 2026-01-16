cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# libserial - Ubuntu'da CMake config dosyası yok, manuel bulma
find_library(LIBSERIAL_LIBRARIES
  NAMES serial
  PATHS 
    /usr/lib
    /usr/lib/aarch64-linux-gnu
    /usr/lib/arm-linux-gnueabihf
    /usr/local/lib
  REQUIRED
)

find_path(LIBSERIAL_INCLUDE_DIRS
  NAMES SerialPort.h
  PATHS 
    /usr/include
    /usr/include/libserial
    /usr/local/include
    /usr/local/include/libserial
  REQUIRED
)

message(STATUS "Found libserial library: ${LIBSERIAL_LIBRARIES}")
message(STATUS "Found libserial headers: ${LIBSERIAL_INCLUDE_DIRS}")

include_directories(include)
include_directories(hardware/include)
include_directories(${LIBSERIAL_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
  hardware/my_robot_system.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# libserial manuel link
target_link_libraries(${PROJECT_NAME} ${LIBSERIAL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${LIBSERIAL_INCLUDE_DIRS})

pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY hardware/include/
  DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(
  FILES my_robot_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
  hardware/include
  ${LIBSERIAL_INCLUDE_DIRS}
)
ament_export_libraries(${PROJECT_NAME} ${LIBSERIAL_LIBRARIES})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()
