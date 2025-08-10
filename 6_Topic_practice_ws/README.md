# Topic Practice

## 创建自定义消息接口功能包

```shell
ros2 pkg create interfaces --build-type ament_cmake --dependencies rosidl_default_generators builtin_interfaces --license MIT-0
```

- `builtin_interfaces` 是ROS2中已有的一个消息接口功能包，可以使用其时间接口Time，表示记录信息的时间
- `rosidl_default_generators` 用于将自定义的消息文件转换为C++、Python源码的模块

**创建自定义消息**

```shell
cd interfaces/
mkdir msg && cd msg
touch SystemStatus.msg
```

输入以下消息类型

```msg
builtin_interfaces/Time stamp # 记录时间戳
string host_name 		# 系统名称
float32 cpu_percent 	# CPU使用率
float32 memory_percent 	# 内存使用率
float32 memory_total 	# 内存总量
float32 memory_available 	# 剩余有效内存
float64 net_sent 		# 网络发送数据总量
float64 net_recv 		# 网络接收数据总量
```

[CMakeLists.txt](interfaces/CMakeLists.txt)

```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SystemStatus.msg"
  DEPENDENCIES builtin_interfaces
)

```

[package.xml](interfaces/package.xml)

```xml
  <maintainer email="2035387001@qq.com">whf</maintainer>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <license>MIT-0</license>
```

在`package.xml`中添加`member_of_group`是为了声明该功能包是一个消息接口功能包，方便ROS 2对其做额外处理。

**编译&验证**

```shell
cd 6_Topic_practice_ws/
colcon build
source ./install/setup.bash
```

出现以下内容即成功

```shell
$ ros2 interface show interfaces/msg/SystemStatus 
builtin_interfaces/Time stamp # 记录时间戳
        int32 sec
        uint32 nanosec
string host_name                # 系统名称
float32 cpu_percent     # CPU使用率
float32 memory_percent  # 内存使用率
float32 memory_total    # 内存总量
float32 memory_available        # 剩余有效内存
float64 net_sent                # 网络发送数据总量
float64 net_recv                # 网络接收数据总量
```

## 获取系统消息并发布

**创建功能包**

```shell
ros2 pkg create publisher --build-type ament_python --dependencies rclpy interfaces --license MIT-0
```

[status_pub.py](publisher/publisher/status_pub.py)

略

[setup.py](publisher/setup.py)

```python
'console_scripts': [
    'pub_node = publisher.status_pub:main',
],
```

**编译&验证**

```shell
colcon build
source ./install/setup.bash
ros2 run publisher pub_node
```

终端监控自定义消息

```shell
cd 6_Topic_practice_ws/
source ./install/setup.bash 
ros2 topic echo /sys_status
```

每秒会接收如下内容

```shell
stamp:
  sec: 1754834674
  nanosec: 646687440
host_name: OooO
cpu_percent: 1.5
memory_percent: 46.29999923706055
memory_total: 7793.16796875
memory_available: 4185.77734375
net_sent: 355.9623804092407
net_recv: 324.79864025115967
```

## QT可视化

**创建功能包**

```shell
ros2 pkg create display --build-type ament_cmake --dependencies rclcp interfaces --license MIT-0
```

[status_display.cpp](display/src/status_display.cpp)

Lambda 表达式

Lambda 表达式在表达能力上和仿函数是等价的。编译器一般也是通过自动生成类似仿函数的代码来实现 Lambda 表达式的。上面的例子，用 Lambda 改写如下：

```cpp
auto Plus = [](int a, int b) { return a + b; };
```

一个完整的 Lambda 表达式的组成如下：

```cpp
[ capture-list ] ( params ) mutable(optional) exception(optional) attribute(optional) -> ret(optional) { body } 
```

- capture-list：捕获列表。前面的例子 auto Plus = [](int a, int b) { return a + b; }; 没有捕获任何变量。
- params：和普通函数一样的参数。
- mutable：只有这个 Lambda 表达式是 mutable 的才允许修改按值捕获的参数。
- exception：异常标识。暂时不必理解。
- attribute：属性标识。暂时不必理解。
- ret：返回值类型，可以省略，让编译器通过 return 语句自动推导。
- body：函数的具体逻辑。

本章代码中的 Lambda 表达式

```cpp
subscription_ = this->create_subscription<SystemStatus>(
    "sys_status", 10, [&](const SystemStatus::SharedPtr msg) -> void
    { label_->setText(get_qstr_from_msg(msg)); });
```

这里 `[&]`是隐式引用捕获，它会自动捕获 Lambda 函数体内实际使用到的所有外部变量（按引用方式），而无需开发者逐个显式声明，实际上是 `[&label_]`

[CMakeLists.txt](display/CMakeLists.txt)

```cmake
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(status_display src/status_display.cpp)
target_link_libraries(status_display Qt5::Widgets) # 对于非ROS功能包使用Cmake原生指令进行链接库
ament_target_dependencies(status_display rclcpp interfaces)


install(TARGETS status_display
  DESTINATION lib/${PROJECT_NAME})

```

**编译&验证**

```shell
colcon build
source ./install/setup.bash
ros2 run display status_display
```

新建终端

```shell
source ./install/setup.bash
ros2 run publisher pub_node
```

QT窗口会刷新系统信息