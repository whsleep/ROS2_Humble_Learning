# 建图&导航

安装 `slam-toolbox`

```shell
sudo apt install ros-humble-slam-toolbox
```

## 手动建图

```shell
ros2 launch fishbot_description slam_manual.launch.py
```

## 地图保存

- 新建功能包
  
```shell
ros2 pkg create fishbot_navigation2
```

- 创建 `maps` 文件夹

```shell
mkdir maps
cd maps
```

- 保存地图

```shell
ros2 run nav2_map_server map_saver_cli -f room
```