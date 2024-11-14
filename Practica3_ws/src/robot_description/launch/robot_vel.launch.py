from os.path import join
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
       DeclareLaunchArgument(
            "config_controllers", 
            default_value=join(get_package_share_directory("robot_description"),'config','controllers.yaml'), 
            description="Controller config file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    speed_controller_node = Node(
        package='robot_description',
        executable='speed_controller_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    nodes = [
        speed_controller_node
    ]
    return LaunchDescription(declared_arguments + nodes)
