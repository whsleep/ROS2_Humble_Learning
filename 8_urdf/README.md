# URDF 创建机器人

## URDF 使用

- 创建功能包

```shell
ros2 pkg create example_description --build-type ament_cmake --license MIT
```

- 创建 `urdf` 文件夹

集中存储 `urdf` 文件

- `urdf` 业务代码

略，与ROS1中教学类似

- `urdf_to_graphviz` 对 `urdf` 文件进行可视化验证

```shell
urdf_to_graphviz example_description/urdf/first_robot.urdf 
```

导出结果为 [robot_name.pdf](/8_urdf/robot_name.pdf)

- `rviz` 可视化

创建 `launch` 文件夹集中处理launch文件

`CMakeList.txt` 中加入

```cmake
install(DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)
```

将 `urdf` 和 `launch` 拷贝到 `install` 下的 `share` 文件夹

- 编写 `launch` 启动

[dis_robot.launch.py](/8_urdf/example_description/launch/dis_robot.launch.py)

## XACRO 简化

- 安装 `XACRO` 验证包

```shell
sudo apt install ros-$ROS_DISTRO-xacro
```

- 检查 `xacro` 文件并转换为 `urdf` 

```shell
xacro example_description/urdf/first_robot.urdf.xacro 
```

- 修改 `launch` 文件打开方式

```shell
# 通过文件路径读取内容
substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('urdf_path')])
robot_description_content = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)
```

## GAZEBO 仿真

安装信息包

```shell
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs
```

安装模型包

```shell
cd ~/.gazebo/
git clone https://gitee.com/ohhuo/gazebo_models.git
mv gazebo_models/ models
```

如果需要机器人模型，在 `CMakeList.txt` 中加入以下代码，将 `meshes` 文件加入少 `install` 目录下

```cmake
install(DIRECTORY urdf launch world meshes
  DESTINATION share/${PROJECT_NAME}
)
```

注意：`launch.py` 文件要主动设置 `GAZEBO_MODEL_PATH` 不然无法加载 ".STL" 模型文件

```python
# 修改 gazebo 模型路径
pkg_share = os.pathsep + os.path.join(get_package_prefix('example_description'), 'share')
if 'GAZEBO_MODEL_PATH' in os.environ:
  os.environ['GAZEBO_MODEL_PATH'] += pkg_share
else:
  os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share
print(os.environ['GAZEBO_MODEL_PATH'])
```


