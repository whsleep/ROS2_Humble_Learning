import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取默认 urdf 路径
    urdf_package_path = get_package_share_directory('example_description')
    default_urdf_path = os.path.join(urdf_package_path, 'urdf', 'r550.xacro')
    print(default_urdf_path)
    # 声明urdf参数
    action_declare_urdf_path = launch.actions.DeclareLaunchArgument(
        name='urdf_path',
        default_value=default_urdf_path,
        description='Absolute path to robot urdf file'
    )
    # 通过文件路径读取内容
    substitutions_command_result = launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('urdf_path')])
    robot_description_content = launch_ros.parameter_descriptions.ParameterValue(substitutions_command_result, value_type=str)
    # 启动机器人状态发布节点
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description_content}],
    )
    # 关节状态发布
        # 启动机器人状态发布节点
    action_joint_state_publisher = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )
    # 启动 rviz
    action_rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # Create and return the launch description
    return launch.LaunchDescription([
        action_declare_urdf_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])