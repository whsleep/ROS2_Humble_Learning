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

## 导航

- 插件安装

```shell
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

- 参数设置

教程中的参数需要进行以下修改

```yaml
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: True
      min_vel_x: 0.0
      min_vel_y: -0.01
      max_vel_x: 1.0
      max_vel_y: 0.01 # Holonomic
      max_vel_theta: 2.0
      min_speed_xy: 0.05
      max_speed_xy: 0.5
      min_speed_theta: 0.0
      # Increase stopped velocity threshold to avoid false detection of stopped state.
      # Adjusted for smoother operation in holonomic or high-precision robots.
      acc_lim_x: 1.0
      acc_lim_y: 0.01 # Holonomic
      acc_lim_theta: 1.5
      decel_lim_x: -1.0
      decel_lim_y: -0.2 # Holonomic
      decel_lim_theta: -1.5
      vx_samples: 20
      vy_samples: 5 # Holonomic
      vtheta_samples: 20
      sim_time: 1.7
      linear_granularity: 0.05
      angular_granularity: 0.025
      transform_tolerance: 0.2
      xy_goal_tolerance: 0.25
      trans_stopped_velocity: 0.25
      short_circuit_trajectory_evaluation: True
      stateful: True
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"] # Holonomic
      BaseObstacle.scale: 0.02
      PathAlign.scale: 48.0
      PathAlign.forward_point_distance: 0.1
      GoalAlign.scale: 32.0
      GoalAlign.forward_point_distance: 0.1
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 64.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

还有

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: True
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

编译运行

```shell
colcon build
source install/setup.bash 
ros2 launch fishbot_description gazebo_sim.launch.py
# 新开一个终端
ros2 launch fishbot_navigation2 navigation2.launch.py
```