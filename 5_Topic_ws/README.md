# Topic

## python

### 创建功能包

```shell
cd 5_Topic_ws/
ros2 pkg create demo_python_topic --build-type ament_python --dependencies rclpy geometry_msgs --license MIT-0
```

其中 `geometry_msgs` 是接下来要用到的消息类型

这里选择 `turtlesim` 作为前端模拟，发布 `/turtle1/cmd_vel` 话题控制乌龟运动


### 添加代码及配置

创建 [cmd_pub.py](demo_python_topic/demo_python_topic/cmd_pub.py)


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

```

新建一个终端，输入

```shell
ros2 run turtlesim turtlesim_node
```

不出意外可以看到乌龟做圆周运动