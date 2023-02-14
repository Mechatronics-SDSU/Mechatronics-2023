from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='ahrs_node', executable='ahrs_exec',
            ),
        launch_ros.actions.Node(
            package='pid_node', executable='pid_exec', output='screen' 
            ),        
        # launch_ros.actions.Node(
        #   package='vision_node', executable='vision_exec'),
    ])