cmake_minimum_required(VERSION 3.8)
project(radar_obs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ulisse_msgs REQUIRED)

## Executables
add_executable(radar src/publisher.cpp)
ament_target_dependencies(radar rclcpp std_msgs ulisse_msgs ctrl_toolbox)
target_link_libraries(radar ctrl_toolbox)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

target_include_directories(radar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(radar PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS radar
  DESTINATION lib/${PROJECT_NAME})


ament_package()
