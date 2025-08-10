# 面对对象编程

## 目标

**创建局部规划器基类，包含以下接口**

- 是否抵达目标 `isGoalReached()`

**创建TEB,MPC派生类**

## python

### 创建功能包

```shell
cd 3_Package/
ros2 pkg create --build-type ament_python py_pkg
```

首先写基类 [_interface.py](py_pkg/py_pkg/_interface.py)

包含

- 初始化 `__init__()`
- 是否抵达目标 `isGoalReached()`

可直接使用以下指令测试是否正确

```shell
cd py_pkg/py_pkg/
python3 _interface.py 
```

输出以下内容即可

```shell
----------------
启用规划器基类接口: PlannerInterface
start: [0. 1.], goal: [2. 2.]
----------------
距离小于1.0: False
距离小于3.0: True
```

派生类 [TebPlanner.py](py_pkg/py_pkg/TebPlanner.py)


继承了 `PlannerInterface` 基类，

在初始化中，首先需要调用 `super().__init__(start, goal)` 调用当前类的父类（基类）的 __init__ 方法

使用以下指令测试

```shell
cd py_pkg/py_pkg/
python3 TebPlanner.py 
```

输出以下内容即正常

```shell
----------------
启用规划器基类接口: PlannerInterface
start: [0. 1.], goal: [2. 2.]
----------------
----------------
启用 Teb 接口
----------------
距离小于1.0: False
距离小于3.0: True
```

**上述两个文件用于理解类和继承**

### 节点继承

[person_node.py](py_pkg/py_pkg/person_node.py)

继承 `ROS` 的节点类

进行以下修改

[setup.py](py_pkg/setup.py) 添加

```python
'console_scripts': [
        'python_pkg_node = py_pkg.person_node:main',
],
```

[package.xml](py_pkg/package.xml) 添加

```shell
<depend>rclpy</depend>
```

**编译运行**

```shell
colcon build
source install/setup.bash
ros2 run py_pkg python_pkg_node
```

输出以下内容即继承成功

```shell
----------------
启用 ['P_Node'] 节点
包含 Teb 规划器
----------------
[INFO] [1754490961.993274042] [P_Node]: 距离大于 1.0
```

## cpp

### 创建功能包

```shell
cd 4_Oop/
ros2 pkg create --build-type ament_cmake cpp_pkg
```

### 添加代码及配置

在 `cpp_pkg/src` 添加 [person_node.cpp](cpp_pkg/src/person_node.cpp)

**配置CmakeList.txt**

```Cmake
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
```

**添加依赖信息**

在 [package.xml](cpp_pkg/package.xml) 中添加 `rclcpp` 依赖

```python
  <depend>rclcpp</depend>
```

### 编译运行

```shell
colcon build
source install/setup.bash
ros2 run cpp_pkg cpp_node
```