import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import os
import xacro

def generate_launch_description():
    # 获取默认 urdf 文件夹路径
    urdf_package_path = get_package_share_directory('example_description')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'r550.xacro')
    # 获取默认 world 文件夹路径
    default_gazebo_world_path = os.path.join(urdf_package_path, 'world', 'world.world')
    print(default_urdf_path)
    # 修改 gazebo 模型路径
    pkg_share = os.pathsep + os.path.join(get_package_prefix('example_description'), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    else:
        os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + pkg_share
    print(os.environ['GAZEBO_MODEL_PATH'])
    # 读取 xacro 文件
    doc = xacro.parse(open(default_urdf_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
	# 启动机器人状态发布节点
    node_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
	# 把机器人模型加载到gazebo
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', 'robot_gazebo'],
        output='screen'
    )
    # gazebo 启动
    action_launch_robot = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments=[('world', default_gazebo_world_path), ('verbose', 'true')]
    )
    # 启动控制器的节点
    start_controllers = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",  # 关节状态广播器
            "mecanum_controller",        # 麦轮控制器（替换为你的控制器名）
        ]
    )
    # Create and return the launch description
    return launch.LaunchDescription([
        action_launch_robot,
        node_robot_state_publisher,
        spawn_entity,
        start_controllers
    ])