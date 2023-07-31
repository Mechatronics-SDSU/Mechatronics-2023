"""
    @author Conner Sommerfield - Zix on Discord
    Launch file for all ROS nodes of the Robot
    Currently will turn on 
        - AHRS Orientation node
        - CAN Driver node
        - DVL Velocity Node
        - MS5837 Depth Node
        - PID Controller Node
        - Specific Zed Position Node (filters data from Zed Node for position data)
        - Zed Node which contains various topics
    
    Run using ros2 launch launch.py in terminal (make sure you're in the src/ folder and that you've sourced)
"""

from launch import LaunchDescription
import launch_ros.actions
""" """ """ """ """ """ """ """ """ """ """ """  """ """ """ """ """ """ """ """ """ """ """ """

def generate_launch_description():
    """ 
    This is what Ros2 will actually call when you run the launch file,
    all the nodes are placed here as well as Zed Node launch arguments
    """
    return LaunchDescription([
         # launch_ros.actions.Node(
        #     package='a50_node', executable='a50_exec', output='screen'
        #     ),
        # launch_ros.actions.Node(
        #     package='ahrs_node', executable='ahrs_exec', output='screen'
        #     ),
        # launch_ros.actions.Node(
        #     package='brain_node', executable='brain_exec', output='screen' 
        #     ),
        # launch_ros.actions.Node(
        #     package='controller_node', executable='controller_exec', output='screen', 
        #     parameters= [
        #                     {"thrust_mapper": "percy"}
        #                 ]
        #     ),  
        # launch_ros.actions.Node(
        #     package='current_state_node', executable='current_state_exec', output='screen',
        #     parameters= [
        #                     {"use_position_tracking": True},
        #                     {"use_orientation_tracking": True},
        #                 ]
        #     ),  
        # launch_ros.actions.Node(
        #     package='gui_node', executable='gui_exec', output='screen'
        #     ),  
        # launch_ros.actions.Node(
        #     package='joy', executable='joy_node', output='screen'
        #     ),
        launch_ros.actions.Node(
            package='launcher_node', executable='launcher_exec', output='screen'
            ),
        # launch_ros.actions.Node(
        #     package='mediator_node', executable='mediator_exec', output='screen' 
        #     ),  
        # launch_ros.actions.Node(
        #     package='pid_node', executable='pid_exec', output='screen',
        #     parameters= [
        #                     {"thrust_mapper": "percy"}
        #                 ]
        #     ),    
        # launch_ros.actions.Node(
        #     package='test_node', executable='test_exec', output='screen' 
        #     ),    
        launch_ros.actions.Node(
            package='unified_can_driver', executable='unified_can_driver', output='screen',         
            parameters= [
                            {"can_bus_interface": "can0"},
                            {"do_module_polling": False},
                            {"module_bitfield": 0}
                        ]
            ),
        # launch_ros.actions.Node(
        #     package='zed_orientation_node', executable='zed_orientation_exec' 
        #     ),   
        # launch_ros.actions.Node(
        #     package='zed_pos_node', executable='zed_pos_exec' 
        #     ),   
        # launch_ros.actions.Node(
        #     package='zed_vision_node', executable='zed_vision_exec' 
        #     ),   
        ])         
    
        





    
