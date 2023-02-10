from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='pid_controller', executable='pid_controller', 
            ),        
        launch_ros.actions.Node(
            package='ahrs_node', executable='ahrs',
            ),
        
        # launch_ros.actions.Node(
        #     namespace= "dummy_brain", package='dummy_brain', executable='dummy_brain', output='screen'),
    ])