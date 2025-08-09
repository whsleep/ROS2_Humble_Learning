# ROS2节点测试

## py节点

```shell
python3 2_Node/py_node.py
```

新建终端输入

```shell
ros2 node list
```

查看当前节点是否创建成功

## cpp节点测试

```shell
cd 2_Node/build
cmake ..
make
./cpp_node
```

⚠️注意：需要在 [CMakeLists.txt](CMakeLists.txt) 中设置为 `C++17` 标准