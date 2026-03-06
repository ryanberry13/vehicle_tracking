cmake_minimum_required(VERSION 3.8)
project(vehicle_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)


add_executable(vehicle_tracking_node
        src/main.cpp)
target_include_directories(vehicle_tracking_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(vehicle_tracking_node rclcpp px4_msgs sensor_msgs)
target_link_libraries(vehicle_tracking_node px4_ros2_cpp::px4_ros2_cpp rclcpp::rclcpp)

add_executable(simulated_vehicle_node src/simulated_vehicle_node.cpp)
ament_target_dependencies(simulated_vehicle_node rclcpp nav_msgs sensor_msgs geometry_msgs)

# GeographicLib (module-mode): use variables, not an imported target
target_include_directories(simulated_vehicle_node PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(simulated_vehicle_node ${GeographicLib_LIBRARIES})

add_executable(path_generation_node src/path_generation_node.cpp src/path_math.cpp)
ament_target_dependencies(path_generation_node rclcpp px4_msgs nav_msgs sensor_msgs geometry_msgs px4_ros2_cpp)

target_include_directories(path_generation_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(path_generation_node PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(path_generation_node ${GeographicLib_LIBRARIES})

install(TARGETS
        vehicle_tracking_node
        simulated_vehicle_node
        path_generation_node
        DESTINATION lib/${PROJECT_NAME})

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
