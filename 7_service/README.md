# 服务通信

## ROS2小海龟为例

终端启动海龟模拟器

```shell
ros2 run turtlesim turtlesim_node
```

查看海龟模拟器所使用的服务列表

```shell
ros2 service list -t
```

如下：

```shell
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
```

左半部分为服务名称，又半部分为服务传递信息的类型

可通过以下指令查看

```shell
whf@OooO:~$ ros2 interface show turtlesim/srv/Spawn
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

服务信息分为两部分

- 请求 [request]

```shell
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
```

- 响应 [response]

```shell
string name
```

使用指令调用上述服务

```shell
(base) whf@OooO:~$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 3, y: 4}"
requester: making request: turtlesim.srv.Spawn_Request(x=3.0, y=4.0, theta=0.0, name='')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

请求在 `{x: 3, y: 4}` 处生成一个小海龟，可以看到此处发生了响应 `name='turtle2'`

> 同样可以调用 `rqt` 进行可视化的服务请求操作

## 自定义服务消息创建

- 在 `7_service` 下创建消息包 `face_service_type`

```shell
ros2 pkg create face_service_type --dependencies sensor_msgs rosidl_default_generators --license MIT
```

同时创建 `srv` 文件夹用于存放自定义服务消息

在 `srv` 下创建 `FaceDetector.srv` 输入以下内容

```
sensor_msgs/Image image
---
int16 number
float32 use_time
int32[] top
int32[] right
int32[] bottom
int32[] left
```

- `CMakeLists.txt` 添加以下内容

```shell
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FaceDetector.srv"
  DEPENDENCIES sensor_msgs
)
```

- `package.xml`

```shell
<member_of_group>rosidl_interface_packages</member_of_group>
```

构建项目

```shell
colcon build
source ./install/setup.bash
```

查看是否定义成功

```shell
whf@OooO:~/Document/Basic_Learn/7_service$ ros2 interface show face_service_type/srv/FaceDetector 
sensor_msgs/Image image
---
int16 number
float32 use_time
int32[] top
int32[] right
int32[] bottom
int32[] left
```

## 服务端

- 安装依赖

```shell
pip install face_recognition
```

- 创建功能包

```shell
ros2 pkg create demo_python_service --build-type ament_python --dependencies rclpy face_service_type --license MIT
```

- 修改 `setup.py` 定位图片

```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name+"resource", ['resource/zidane.jpg'])
],
```

使其拷贝一份到 `install` 目录下

- 编写业务代码

[face_detect_node.py](/7_service/demo_python_service/demo_python_service/face_detect_node.py)

在 `setup.py` 内提供 `ros2` 调用接口

```python
entry_points={
    'console_scripts': [
        'face_detect_node = demo_python_service.face_detect_node:main',
    ],
},
```

- 运行代码及测试

```shell
ros2 run demo_python_service face_detect_node
# 另开一终端进行测试，需要先 source 
ros2 service call /face_detect face_service_type/srv/FaceDetector "{}"
```

结果如下：

```shell
whf@OooO:~/Document/Basic_Learn/7_service$ ros2 run demo_python_service face_detect_node 
[INFO] [1759584526.760845091] [face_detect_node]: Face Detect Service is Ready.
[INFO] [1759584530.986052356] [face_detect_node]: request 图像为空，使用默认图像
[INFO] [1759584530.986282839] [face_detect_node]: 图片加载完成,开始检测人脸...
[INFO] [1759584531.273016142] [face_detect_node]: 人脸检测完成,耗时 0.2864 秒
[INFO] [1759584531.273258056] [face_detect_node]: 检测到 1 张人脸
[INFO] [1759584531.273433035] [face_detect_node]: 检测结果返回完成
```

## 客户端

- 业务代码


[face_detect_client_node.py](/7_service/demo_python_service/demo_python_service/face_detect_client_node.py)

在 `setup.py` 内提供 `ros2` 调用接口

```python
entry_points={
    'console_scripts': [
        'face_detect_node = demo_python_service.face_detect_node:main',
        'face_detect_client_node = demo_python_service.face_detect_client_node:main',
    ],
},
```

- 运行及测试

```shell
# 服务端
ros2 run demo_python_service face_detect_node
# 客户端
ros2 run demo_python_service face_detect_client_node
```

# 参数

## 参数声明

在 [face_detect_node.py](/7_service/demo_python_service/demo_python_service/face_detect_node.py)

```shell
self.declare_parameter("number_of_times_to_upsample", 1)
self.number_of_times_to_upsample = 1
self.declare_parameter("model", "hog")
self.model = "hog"
```

运行 

```shell
ros2 run demo_python_service face_detect_node 
```

查看是否声明成功

```shell
whf@OooO:~/Document$ ros2 param list
/face_detect_node:
  model
  number_of_times_to_upsample
  use_sim_time
```

获取/设置参数

```shell
whf@OooO:~/Document$ ros2 param get /face_detect_node number_of_times_to_upsample
Integer value is: 1
whf@OooO:~/Document$ ros2 param get /face_detect_node model
String value is: hog
whf@OooO:~/Document$ ros2 param set /face_detect_node number_of_times_to_upsample 5
Set parameter successful
whf@OooO:~/Document$ ros2 param get /face_detect_node number_of_times_to_upsample
Integer value is: 5
```

## 动态更新参数

动态参数更新相当于服务通信, 对于传入参数在节点内使用一个回调函数进行处理

```python
# 绑定回调函数
self.add_on_set_parameters_callback(self.param_callback)

# 参数修改回调函数
    def param_callback(self, params):
        for param in params:
            if param.name == "number_of_times_to_upsample":
                self.number_of_times_to_upsample = param.value
                self.get_logger().info(f"number_of_times_to_upsample set to {self.number_of_times_to_upsample}")
            elif param.name == "model":
                self.model = param.value
                self.get_logger().info(f"model set to {self.model}")
        # 返回成功信息
        return SetParametersResult(successful=True)
```

运行测试

```shell
# 运行
ros2 run demo_python_service face_detect_node 
# 修改参数
ros2 param set /face_detect_node number_of_times_to_upsample 5
# [INFO] [1759651377.032233800] [face_detect_node]: number_of_times_to_upsample set to 5

ros2 param set /face_detect_node model cnn
# [INFO] [1759651493.653109824] [face_detect_node]: model set to cnn
```

## Launch 启动多个节点

创建 `launch` 文件夹

新建 `launch.py` 文件

> cpp 版本在 `CMakeList.txt` 中加入以下内容
> ```CMake
> install(DIRECTORY launch
> DESTINATION lib/${PROJECT_AME}
> )
> ```

`setup.py` 引入 

```shell
from glob import glob
```

并在 `data_files` 加入 `launch` 目录

```python
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/launch", glob('launch/*.launch.py')),
    ],
```

编译运行验证

```shell
ros2 launch demo_python_service demo.launch.py
```