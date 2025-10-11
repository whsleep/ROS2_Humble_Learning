import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取包路径
    fishbot_description_path = get_package_share_directory('fishbot_description')
    slam_toolbox_path = get_package_share_directory('slam_toolbox')
    
    # 声明参数 - 正确的参数声明方式
    declare_use_sim_time_arg = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='是否使用仿真时间'
    )
    
    # 声明SLAM参数文件路径 - 修正错误的地方
    declare_slam_params_file_arg = launch.actions.DeclareLaunchArgument(
        'slam_params_file',
        default_value=[
            launch.substitutions.TextSubstitution(text=slam_toolbox_path),
            '/config/mapper_params_online_async.yaml'
        ],
        description='SLAM参数文件路径'
    )
    
    # 使用LaunchConfiguration获取参数值（不包含default_value）
    slam_params_file = launch.substitutions.LaunchConfiguration('slam_params_file')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    
    # 启动SLAM节点 (使用slam_toolbox的异步建图)
    slam_node = launch_ros.actions.Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 启动键盘控制节点
    teleop_node = launch_ros.actions.Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # 使用xterm显示键盘控制界面
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 启动RViz可视化
    rviz_config_file = fishbot_description_path + '/config/rviz/slam.rviz'
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # 包含Gazebo和机器人启动文件
    gazebo_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            fishbot_description_path,
            '/launch/gazebo_sim.launch.py'  # 替换为你的Gazebo启动文件实际路径
        ])
    )
    
    return launch.LaunchDescription([
        declare_use_sim_time_arg,
        declare_slam_params_file_arg,
        gazebo_launch,
        slam_node,
        teleop_node,
        rviz_node
    ]) 