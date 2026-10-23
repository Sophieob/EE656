import os

from ament_index_python.packages import get_package_share_directory 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    hardware_type_arg = DeclareLaunchArgument(
        'hardware_type',
        default_value='actual',
        description='actual = real robot, fake = simulated robot in RViz',
    )
    hardware_type = LaunchConfiguration('hardware_type')

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

    dealer_node = Node(
        package='rx200_dealer_client',
        executable='rx200_dealer',
        output='screen',
    )

    return LaunchDescription([
        hardware_type_arg,
        interbotix_moveit,
        dealer_node,
    ])
