cmake_minimum_required(VERSION 3.5)
project(bpl_waypoint_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(bpl_interfaces REQUIRED)


#include_directories(/usr/include/mavsdk)
include_directories(include)

add_library(waypoint_control_component SHARED
  src/waypoint_control.cpp
)

rclcpp_components_register_nodes(waypoint_control_component PLUGIN "WaypointControl" EXECUTABLE waypointControl)

target_compile_definitions(waypoint_control_component
  PRIVATE "UARS_WAYPOINT_CONTROL_COMPONENT_BUILDING_DLL"
)

ament_target_dependencies(waypoint_control_component 
rclcpp 
rclcpp_components
std_srvs 
std_msgs
nav_msgs
geometry_msgs
mavros_msgs
sensor_msgs
Eigen3
bpl_interfaces
)

add_executable(waypoint_control src/waypoint_control_node.cpp src/waypoint_control.cpp)
ament_target_dependencies(waypoint_control rclcpp)
target_link_libraries(waypoint_control
    waypoint_control_component
)
install(TARGETS waypoint_control waypoint_control_component DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
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
