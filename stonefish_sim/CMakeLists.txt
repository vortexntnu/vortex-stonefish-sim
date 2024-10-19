cmake_minimum_required(VERSION 3.8)
project(stonefish_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(stonefish_ros2 REQUIRED)

install(DIRECTORY
  launch
  scenarios
  data
  metadata
  drones
  tasks
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
