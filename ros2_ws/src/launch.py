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
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)

from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    TextSubstitution
)


"""    Just some adminsistrative stuff first for the ZED Node   """

# ZED Configurations to be loaded by ZED Node
default_config_common = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'config',
    'common.yaml'
)

# URDF/xacro file to be loaded by the Robot State Publisher node
default_xacro_path = os.path.join(
    get_package_share_directory('zed_wrapper'),
    'urdf',
    'zed_descr.urdf.xacro'
)

def parse_array_param(param):
    str = param.replace('[', '')
    str = str.replace(']', '')
    arr = str.split(',')
    return arr

""" """ """ """ """ """ """ """ """  """ """ """ """ """ """ """ """ """

def launch_setup(context, *args, **kwargs):

    """
    Sets a bunch of parameter names to be used later in the launch arguments 
    """
    wrapper_dir = get_package_share_directory('zed_wrapper')

    # Launch configuration variables
    svo_path = LaunchConfiguration('svo_path')

    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')

    node_name = LaunchConfiguration('node_name')

    config_common_path = LaunchConfiguration('config_path')

    zed_id = LaunchConfiguration('zed_id')
    serial_number = LaunchConfiguration('serial_number')

    base_frame = LaunchConfiguration('base_frame')
    cam_pose = LaunchConfiguration('cam_pose')

    publish_urdf = LaunchConfiguration('publish_urdf')
    publish_tf = LaunchConfiguration('publish_tf')
    publish_map_tf = LaunchConfiguration('publish_map_tf')
    xacro_path = LaunchConfiguration('xacro_path')

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)

    if (camera_name_val == ""):
        camera_name_val = camera_model_val

    config_camera_path = os.path.join(
        get_package_share_directory('zed_wrapper'),
        'config',
        camera_model_val + '.yaml'
    )

    # Convert 'cam_pose' parameter
    cam_pose_str = cam_pose.perform(context)
    cam_pose_array = parse_array_param(cam_pose_str)

    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(publish_urdf),
        package='robot_state_publisher',
        namespace=camera_name_val,
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_name_val, ' ',
                    'camera_model:=', camera_model_val, ' ',
                    'base_frame:=', base_frame, ' ',
                    'cam_pos_x:=', cam_pose_array[0], ' ',
                    'cam_pos_y:=', cam_pose_array[1], ' ',
                    'cam_pos_z:=', cam_pose_array[2], ' ',
                    'cam_roll:=', cam_pose_array[3], ' ',
                    'cam_pitch:=', cam_pose_array[4], ' ',
                    'cam_yaw:=', cam_pose_array[5]
                ])
        }]
    )

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package='zed_wrapper',
        namespace=camera_name_val,
        executable='zed_wrapper',
        name=node_name,
        output='screen',
        #prefix=['xterm -e valgrind --tools=callgrind'],
        #prefix=['xterm -e gdb -ex run --args'],
        parameters=[
            # YAML files
            config_common_path,  # Common parameters
            config_camera_path,  # Camera related parameters
            # Overriding
            {
                'general.camera_name': camera_name_val,
                'general.camera_model': camera_model_val,
                'general.svo_file': svo_path,
                'pos_tracking.base_frame': base_frame,
                'general.zed_id': zed_id,
                'general.serial_number': serial_number,
                'pos_tracking.publish_tf': publish_tf,
                'pos_tracking.publish_map_tf': publish_map_tf,
                'pos_tracking.publish_imu_tf': publish_tf
            }
        ]
    )

    return [
        rsp_node,
        zed_wrapper_node
    ]

""" """ """ """ """ """ """ """ """ """ """ """  """ """ """ """ """ """ """ """ """ """ """ """

def generate_launch_description():
    """ 
    This is what Ros2 will actually call when you run the launch file,
    all the nodes are placed here as well as Zed Node launch arguments
    """
    # ****************** WHOLE LOT OF ZED NODE STUFF ******************* #
    camera_model = 'zed2i' 
    return LaunchDescription([
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        DeclareLaunchArgument(
            'camera_name',
            default_value=TextSubstitution(text=""),
            description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`. Leave empty to use the camera model as camera name.'),
        DeclareLaunchArgument(
            'camera_model',
            default_value = 'zed2i',
            description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `zed`, `zedm`, `zed2`, `zed2i`.'),
        DeclareLaunchArgument(
            'node_name',
            default_value='zed_node',
            description='The name of the zed_wrapper node. All the topic will have the same prefix: `/<camera_name>/<node_name>/`'),
        DeclareLaunchArgument(
            'config_path',
            default_value=TextSubstitution(text=default_config_common),
            description='Path to the YAML configuration file for the camera.'),
        DeclareLaunchArgument(
            'zed_id',
            default_value='0',
            description='The index of the camera to be opened. To be used in multi-camera rigs.'),
        DeclareLaunchArgument(
            'serial_number',
            default_value='0',
            description='The serial number of the camera to be opened. To be used in multi-camera rigs. Has priority with respect to `zed_id`.'),
        DeclareLaunchArgument(
            'publish_urdf',
            default_value='true',
            description='Enable URDF processing and starts Robot State Published to propagate static TF.'),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description='Enable publication of the `odom -> base_link` TF.'),
        DeclareLaunchArgument(
            'publish_map_tf',
            default_value='true',
            description='Enable publication of the `map -> odom` TF. Note: Ignored if `publish_tf` is False.'),
        DeclareLaunchArgument(
            'xacro_path',
            default_value=TextSubstitution(text=default_xacro_path),
            description='Path to the camera URDF file as a xacro file.'),
        DeclareLaunchArgument(
            'svo_path',
            default_value=TextSubstitution(text="live"),
            description='Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`.'),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Name of the base link.'),
        DeclareLaunchArgument(
            'cam_pose',
            default_value='[0.0,0.0,0.0,0.0,0.0,0.0]',
            description='Pose of the camera with respect to the base frame (i.e. `base_link`): [x,y,z,r,p,y]. Note: Orientation in rad.)'),
        # ******************* EVERYTHING BELOW HERE ARE OUR NODES ************** #
        # launch_ros.actions.Node(
        #     package='ahrs_node', executable='ahrs_exec',
        #     ),
        launch_ros.actions.Node(
            package='pid_node', executable='pid_exec', output='screen' 
            ),    
        launch_ros.actions.Node(
            package='can2ros_driver', executable='can2ros_driver'
            ),
        # # launch_ros.actions.Node(
        # #     package='dres_dvl_decode', executable='dres_dvl_decode' 
        # #     ),
        # launch_ros.actions.Node(
        #     package='dres_ms5837_decode', executable='dres_ms5837_decode'
        #     ),
        launch_ros.actions.Node(
            package='zed_pos_node', executable='zed_pos_exec' 
            ),   
        # launch_ros.actions.Node(
        #     package='zed_vision_node', executable='zed_vision_exec' 
        #     ),   
        OpaqueFunction(function=launch_setup)
    ])# This includes all the parameters from launch_setup function            
    
        





    
