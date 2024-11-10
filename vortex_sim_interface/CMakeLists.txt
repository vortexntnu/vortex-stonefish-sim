cmake_minimum_required(VERSION 3.8)
project(vortex_sim_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(stonefish_ros2 REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(vortex_sim_interface 
  src/vortex_sim_interface.cpp
)

target_include_directories(vortex_sim_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(vortex_sim_interface
 rclcpp 
 std_msgs
 vortex_msgs
 nav_msgs
 geometry_msgs
 tf2_ros
 stonefish_ros2
)

install(TARGETS 
  vortex_sim_interface
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(vortex_sim_interface Eigen3::Eigen)

install(DIRECTORY 
  launch 
  DESTINATION share/${PROJECT_NAME})

ament_package()
