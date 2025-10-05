import launch
import launch_ros

def generate_launch_description(): 
    action_node_service = launch_ros.actions.Node(
            package='demo_python_service',
            executable='face_detect_node',
            name='face_detect_node',
            output='screen',
            parameters=[
                {'number_of_times_to_upsample': 1},
                {'model': 'hog'}
            ]
        )

    action_node_client = launch_ros.actions.Node(
            package='demo_python_service',
            executable='face_detect_client_node',
            name='face_detect_client_node',
            output='screen'
        )

    return launch.LaunchDescription([
        action_node_service,
        action_node_client
    ])