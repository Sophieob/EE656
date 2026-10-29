import os

from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    hardware_type = LaunchConfiguration('hardware_type')

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
        } .items()

    )

    # Starts the dealer node
    dealer_node = Node(
        package='rx200_dealer_client',
        executable='rx200_dealer',
        output='screen',
    )

# Started in this order but run all at the same time. The dealer node waits for MoveIt by itself
    return LaunchDescription([
        hardware_type_arg,
        interbotix_moveit,
        dealer_node,
    ])
