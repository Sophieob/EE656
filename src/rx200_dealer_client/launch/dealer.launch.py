import os

from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, EmitEvent
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ROS2 calls this function when the user runs the ROS2 launch command and it returns a list of things to start
def generate_launch_description():
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='actual',
        description='actual = real robot, fake = simulated robot in RViz',
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='true = Starts the RealSense camera, false = runs without it',
    )
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='true = opens the stacking GUI window',
    )
    hardware_type = LaunchConfiguration('hardware_type')
    use_camera = LaunchConfiguration('use_camera')
    use_gui = LaunchConfiguration('use_gui')

    external_urdf_loc = os.path.join(get_package_share_directory('rx200_dealer_client'), 'urdf', 'rx200_external.urdf.xacro')

    # Runs the Interbotix MoveIt launch file from inside the launch file (Starts the robot driver, controllers, MoveIt's move_group with the /move_actions and RViz)
    interbotix_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('interbotix_xsarm_moveit'),
                'launch',
                'xsarm_moveit.launch.py'
            )
        ),
        launch_arguments={
            'robot_model': 'rx200',
            'hardware_type': hardware_type,
            'external_urdf_loc': external_urdf_loc,
        } .items()

    )
    # Starts the RealSense D415 on the wrist (only if use_camera is true)
    realsense_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_name': 'camera',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true'
        }.items(),
        condition=IfCondition(use_camera),
    )

    # Starts our dealer and cube stacking nodes
    dealer_node = Node(
        package='rx200_dealer_client',
        executable='rx200_dealer',
        output='screen',
    )
    cube_stacking_node = Node(
        package='rx200_dealer_client',
        executable='cube_stacking',
        output='screen',
    )
    # Closing the GUI window shuts everything down
    stacking_gui_node = Node(
        package='rx200_dealer_client',
        executable='stacking_gui',
        output='screen',
        condition=IfCondition(use_gui),
        on_exit=EmitEvent(event=Shutdown(reason='Stacking GUI Closed')),
    )

# Started in this order but run all at the same time. The dealer node waits for MoveIt by itself
    return LaunchDescription([
        hardware_type_arg,
        use_camera_arg,
        use_gui_arg,
        interbotix_moveit,
        realsense_camera,
        dealer_node,
        # cube_stacking_node, #TODO
        # stacking_gui_node, #TODO
    ])
