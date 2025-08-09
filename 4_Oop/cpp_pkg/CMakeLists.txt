cmake_minimum_required(VERSION 3.8)
project(cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找必要的 ROS 2 组件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 查找 Eigen 库
find_package(Eigen3 REQUIRED)
 
#头文件加载路径
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# 定义可执行文件
add_executable(person_node src/person_node.cpp)

# 为可执行文件链接 ROS 2 依赖和 Eigen 库
ament_target_dependencies(person_node rclcpp)

# 安装可执行文件
install(TARGETS person_node DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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
