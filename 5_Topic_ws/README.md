# Topic

## python

### 创建功能包

```shell
cd 5_Topic_ws/
ros2 pkg create demo_python_topic --build-type ament_python --license MIT-0
```

这里选择 `turtlesim` 作为前端模拟，发布 `/turtle1/cmd_vel` 话题控制乌龟运动


### 添加代码及配置

创建 [cmd_pub.py](demo_python_topic/demo_python_topic/cmd_pub.py)
创建 [pub_goal.py](demo_python_topic/demo_python_topic/pub_goal.py)

**添加依赖**

```xml
  <depend>rclpy</depend>
  <depend>turtlesim</depend>
```

**入口点配置**

在 [setup.py](demo_python_topic/setup.py) 中的入口点添加配置

```python
entry_points={
    'console_scripts': [
        'python_pkg_node = demo_python_topic.cmd_pub:main',
    ],
},
```

表示当用户在命令行中运行 `python_pkg_node` 命令时，会执行 `demo_python_topic` 包中 `cmd_pub` 模块的 `main` 函数

### 编译运行

```shell
colcon build
source install/setup.bash
ros2 run demo_python_topic sub_node
```

新建一个终端，发布目标

```shell
ros2 run demo_python_topic pub_node
```

新建一个终端，启动小乌龟

```shell
ros2 run turtlesim turtlesim_node
```

## cpp

### 创建功能包

```shell
cd 5_Topic_ws/
ros2 pkg create --build-type ament_cmake cpp_pkg
```

### 添加代码及配置

在 `cpp_pkg/src` 添加 [person_node.cpp](cpp_pkg/src/person_node.cpp)

**配置CmakeList.txt**

```Cmake
# 查找必要的 ROS 2 组件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# 定义可执行文件
add_executable(traj_node src/traj_node.cpp)
# 为可执行文件链接 ROS 2 依赖
ament_target_dependencies(
  traj_node
  rclcpp
  geometry_msgs
  turtlesim
)
```


**添加依赖信息**

在 [package.xml](cpp_pkg/package.xml) 中添加 `rclcpp` 依赖

```python
  <depend>rclcpp</depend>
  <depend>geometry_msgs</depend>
  <depend>turtlesim</depend>
```

### 编译运行

```shell
colcon build
source install/setup.bash
ros2 run cpp_pkg traj_node
```

新建一个终端，发布目标

```shell
ros2 run demo_python_topic pub_node
```

新建一个终端，启动小乌龟

```shell
ros2 run turtlesim turtlesim_node
```