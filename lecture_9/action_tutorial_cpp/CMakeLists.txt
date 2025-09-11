cmake_minimum_required(VERSION 3.8)
project(action_tutorial_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_action REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)


add_executable(add_action_client src/add_client.cpp)
ament_target_dependencies(add_action_client rclcpp_action rclcpp my_action)

add_executable(add_action_server src/add_server.cpp)
ament_target_dependencies(add_action_server rclcpp_action rclcpp my_action)

add_executable(move_action_client src/move_action_client.cpp)
ament_target_dependencies(move_action_client rclcpp rclcpp_action turtlesim my_action)

add_executable(move_action_server src/move_action_server.cpp)
ament_target_dependencies(move_action_server rclcpp rclcpp_action turtlesim my_action)

install(TARGETS
  add_action_client
  add_action_server
  move_action_client
  move_action_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

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
