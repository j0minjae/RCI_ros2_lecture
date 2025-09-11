cmake_minimum_required(VERSION 3.8)
project(hello_ros_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# amnt 빌드할 때 필요 dependencies 지정
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 실행 파일 지정 및 dependencies
add_executable(hello_world_node src/practice2.cpp)
ament_target_dependencies(hello_world_node rclcpp)
add_executable(logging_node src/practice3.cpp)
ament_target_dependencies(logging_node rclcpp)

# 빌드 대상을 설치(Node, Laucnh ...)
install(TARGETS
  hello_world_node
  logging_node
  DESTINATION lib/${PROJECT_NAME}
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
