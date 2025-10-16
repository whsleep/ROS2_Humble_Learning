# plugin 

安装 `pluginlib`

```shell
sudo apt install ros-humble-pluginlib -y
```

## 自定义控制器 

- 创建 `package`

    ```shell
    ros2 pkg create motion_control_system --dependencies pluginlib --license MIT
    ```

- 创建控制器基类 `motion_control_system::MotionController`

内含两个纯虚函数

```cpp
        virtual void start() = 0;
        virtual void stop() = 0;
```

- 创建派生类 `motion_control_system::SpinMotionController`

    ```cpp
        public:
            void start() override;
            void stop() override;
        /* * cpp * */
        void SpinMotionController::start()
        {
            // 实现旋转运动控制逻辑
            std::cout << "SpinMotionController::start" << std::endl;
        }
        void SpinMotionController::stop()
        {
            // 停止运动控制
            std::cout << "SpinMotionController::stop" << std::endl;
        }
    ```

- 配置 `pluginlib` 的 `xml` 文件

    ```xml
    <library path="spin_motion_controller">
        <class name="motion_control_system/SpinMotionController" type="motion_control_system::SpinMotionController" base_class_type="motion_control_system::MotionController">
        <description>Spin Motion Controller</description>
        </class>
    </library>
    ```

- 修改 [CMakeLists.txt](/10_plugin/src/motion_control_system/CMakeLists.txt)
  
生成动态库

```cmake
include_directories(include)
# 添加库文件
add_library(spin_motion_controller SHARED src/spin_motion_controller.cpp)
ament_target_dependencies(spin_motion_controller  pluginlib )

install(DIRECTORY include/
  DESTINATION include/
)

# 导出插件描述文件
pluginlib_export_plugin_description_file(motion_control_system spin_motion_plugins.xml)
```

- 添加测试代码

[test_plugin.cpp](/10_plugin/src/motion_control_system/src/test_plugin.cpp)

```cpp
    // 判断参数数量是否合法
    if (argc != 2)
        return 0;
    // 通过命令行参数，选择要加载的插件,argv[0]是可执行文件名，argv[1]表示参数名
    std::string controller_name = argv[1];
    // 1.通过功能包名称和基类名称创建控制器加载器
    pluginlib::ClassLoader<motion_control_system::MotionController>
        controller_loader("motion_control_system",
                          "motion_control_system::MotionController");
    // 2.使用加载器加载指定名称的插件，返回的是指定插件类的对象的指针
    auto controller = controller_loader.createSharedInstance(controller_name);
    // 3.调用插件的方法
    controller->start();
    controller->stop();
```

编译后调用指令

```shell
ros2 run motion_control_system test_plugin motion_control_system/SpinMotionController
```

启动，重点是输入参数 `motion_control_system/SpinMotionController`

根据该参数加载对应的派生类

## 自定义导航规划器

- 创建功能包 

```shell
ros2 pkg create nav2_custom_planner --dependencies pluginlib nav2_core --license MIT
```

- 创建 [nav2_custom_planner.hpp](/10_plugin/src/nav2_custom_planner/include/nav2_custom_planner/nav2_custom_planner.hpp)

继承

```cpp
class CustomPlanner : public nav2_core::GlobalPlanner
```

`GlobalPlanner` 中定义了几个纯虚函数

```cpp
class GlobalPlanner
{
public:
  /** 略 **/
  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) = 0;
  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return      The sequence of poses to get from start to goal, if any
   */
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) = 0;
};
```

派生类中需要重写上述四个纯虚函数

- 创建 [nav2_custom_planner.cpp](/10_plugin/src/nav2_custom_planner/src/nav2_custom_planner.cpp)

这里直接使用线性插值计算得到路径点

- 创建 `custom_planner_plugin.xml` 

- 修改 `CMakeList.txt`

```cmake
# 包含头文件目录
include_directories(include)
# 定义库名称
set(library_name ${PROJECT_NAME}_plugin)
# 创建共享库
add_library(${library_name} SHARED  src/nav2_custom_planner.cpp)
# 指定库的依赖关系
ament_target_dependencies(${library_name} nav2_core pluginlib)
# 安装库文件到指定目录
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# 安装头文件到指定目录
install(DIRECTORY include/
  DESTINATION include/ )
# 导出插件描述文件
pluginlib_export_plugin_description_file(nav2_core custom_planner_plugin.xml)
```

## 自定义控制器

- 创建 `package`

```shell
ros2 pkg create nav2_custom_controller --build-type ament_cmake --dependencies pluginlib nav2_core --license MIT
```

- 创建 [custom_controller.hpp](/10_plugin/src/nav2_custom_controller/include/nav2_custom_controller/custom_controller.hpp) 继承 `nav2_core::Controller` 父类

有以下几个纯虚函数需要重写

```cpp
  virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) = 0;

  /**
   * @brief Method to cleanup resources.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;

  /**
   * @brief local setPlan - Sets the global plan
   * @param path The global plan
   */
  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  /**
   * @brief Controller computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the current goal checker the task is utilizing
   * @return The best command for the robot to drive
   */
  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) = 0;

  /**
   * @brief Limits the maximum linear speed of the robot.
   * @param speed_limit expressed in absolute value (in m/s)
   * or in percentage from maximum robot speed.
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) = 0;
```

- 创建 [custom_controller.cpp](/10_plugin/src/nav2_custom_controller/src/custom_controller.cpp)

- 创建 [nav2_custom_controller.xml](/10_plugin/src/nav2_custom_controller/nav2_custom_controller.xml) 

- 修改 `CMakeList.txt`

```cmake
# 包含头文件目录
include_directories(include)
# 定义库名称
set(library_name ${PROJECT_NAME}_plugin)
# 创建共享库
add_library(${library_name} SHARED src/custom_controller.cpp)
# 指定库的依赖关系
ament_target_dependencies(${library_name} nav2_core pluginlib)
# 安装库文件到指定目录
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)s
# 安装头文件到指定目录
install(DIRECTORY include/
 DESTINATION include/)
# 导出插件描述文件
pluginlib_export_plugin_description_file(nav2_core nav2_custom_controller.xml)
```

- 配置修改

    [nav2_params.yaml](/10_plugin/src/fishbot_navigation2/config/nav2_params.yaml)

```yaml
    FollowPath:
      plugin: "nav2_custom_controller::CustomController"
      max_linear_speed: 0.1
      max_angular_speed: 1.0
```