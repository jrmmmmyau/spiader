cmake_minimum_required(VERSION 3.5)
project(my_bot)

set(CMAKE_SUPPRESS_REGENERATION true)
add_compile_options(-w)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)


add_library(my_bot SHARED
    src/arduino_hardware.cpp
)


target_include_directories(my_bot PUBLIC include)

ament_target_dependencies(my_bot
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    pluginlib
)

target_link_libraries(my_bot serial)

pluginlib_export_plugin_description_file(hardware_interface arduino_hardware_plugin.xml)

install(TARGETS my_bot
    DESTINATION lib
)

install(DIRECTORY include/
    DESTINATION include
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

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
