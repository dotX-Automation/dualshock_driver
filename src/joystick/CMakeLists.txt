cmake_minimum_required(VERSION 3.8)
project(joystick)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dua_app_management REQUIRED)
find_package(dua_node REQUIRED)
find_package(dua_qos REQUIRED)
find_package(params_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(joystick_msgs REQUIRED)
find_package(Threads REQUIRED)

# Generate parameters source code
generate_init_parameters(
  YAML_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/joystick/params.yaml"
  OUT_FILE "init_parameters.cpp")

add_executable(joystick
  src/joystick/joystick.cpp
  src/joystick/joystick_utils.cpp
  init_parameters.cpp
  src/joystick_app.cpp)

target_include_directories(joystick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  joystick
  dua_app_management
  dua_node
  dua_qos
  joystick_msgs
  params_manager
  rclcpp
  rclcpp_components)

# joystick app
install(TARGETS joystick
  DESTINATION lib/${PROJECT_NAME})

# Config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

# Launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
