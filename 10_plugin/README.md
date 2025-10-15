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

