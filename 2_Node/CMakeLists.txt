cmake_minimum_required(VERSION 3.10)

project(cpp_node)
# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
# 生成可执行文件
add_executable(cpp_node
    cpp_node.cpp
)

find_package(rclcpp REQUIRED)

message(STATUS "rclcpp_INCLUDE_DIRS: ${rclcpp_INCLUDE_DIRS}")
message(STATUS "rclcpp_LIBRARIES: ${rclcpp_LIBRARIES}")
# 查找rclcpp的头文件和库
target_include_directories(cpp_node
    PUBLIC
        ${rclcpp_INCLUDE_DIRS}
)

# 链接rclcpp库到可执行文件
target_link_libraries(cpp_node
    ${rclcpp_LIBRARIES}
)
