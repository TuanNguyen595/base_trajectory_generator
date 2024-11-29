cmake_minimum_required(VERSION 3.8)
project(base_path_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set (dependencies 
  rclcpp
  nav_msgs
  nav2_msgs
  rclcpp_action
  std_msgs
  visualization_msgs
  )
add_executable(base_path_generator_node src/base_path_generator_node.cpp)
target_include_directories(base_path_generator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(base_path_generator_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(base_path_generator_node ${dependencies})

add_executable(path_checker_node 
src/path_checker_node.cpp
src/third_party/navfn.cpp
)
target_include_directories(path_checker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(path_checker_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(path_checker_node ${dependencies})

install(TARGETS base_path_generator_node
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS path_checker_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
