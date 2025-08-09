# Package 编写

## python 

### 创建功能包

```shell
cd 3_Package/
ros2 pkg create --build-type ament_python python_pkg
```

☝️`create` 指令还有其他附加选项，可输入 `ros2 pkg create --help` 查看，创建完成后也可在[package.xml](python_pkg/package.xml) 内修改

### 添加代码及配置

在 `python_pkg/python_pkg` 添加 [py_node.py](python_pkg/python_pkg/py_node.py)

**入口点配置**

在 [setup.py](python_pkg/setup.py) 中的入口点添加配置

```python
entry_points={
    'console_scripts': [
        'python_pkg_node = python_pkg.py_node:main',
    ],
},
```

表示当用户在命令行中运行 `python_pkg_node` 命令时，会执行 `python_pkg` 包中 `py_node` 模块的 `main` 函数

**添加依赖信息**

在 [package.xml](python_pkg/package.xml) 中添加 `rclpy` 依赖

```python
  <depend>rclpy</depend>
```

### 编译运行

```shell
colcon build
source install/setup.bash
ros2 run python_pkg python_pkg_node
```

## cpp

### 创建功能包

```shell
cd 3_Package/
ros2 pkg create --build-type ament_cmake cpp_pkg
```

☝️`create` 指令还有其他附加选项，可输入 `ros2 pkg create --help` 查看，创建完成后也可在[package.xml](cpp_pkg/package.xml) 内修改

### 添加代码及配置

在 `cpp_pkg/src` 添加 [cpp_node.cpp](cpp_pkg/src/cpp_node.cpp)

**配置CmakeList.txt**

```Cmake
# 查找package
find_package(rclcpp REQUIRED)
# 生成可执行文件
add_executable(cpp_node
    src/cpp_node.cpp
)
# 为可执行文件链接依赖库
ament_target_dependencies(cpp_node
    rclcpp
)
# 安装可执行文件
install(TARGETS cpp_node
  DESTINATION lib/${PROJECT_NAME}
)
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