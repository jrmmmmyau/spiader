cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(ending_search src/ending_search.cpp)
ament_target_dependencies(ending_search rclcpp visualization_msgs nav_msgs nav2_msgs rclcpp_action)

install(
  DIRECTORY yaml
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ending_search
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
