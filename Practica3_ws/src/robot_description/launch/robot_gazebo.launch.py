from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_package = get_package_share_directory('msr_world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    position_z = LaunchConfiguration("position_z")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot_name = LaunchConfiguration("robot_name", default="")
    gui_rviz = LaunchConfiguration("gui_rviz", default='true')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', PythonExpression(['"/robot_description"']), 
            '-entity', PythonExpression(['"', robot_name, '"']), 
            '-x', position_x,
            '-y', position_y,
            '-z', position_z,
            '-Y', orientation_yaw
        ]
    )

    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={"verbose": "false"}.items(),
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "rviz",  "view_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=[PythonExpression(['"',join(world_package, 'worlds'),'" + "/sand.world"']),'']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time),
        DeclareLaunchArgument("position_x", default_value="-1.0"),
        DeclareLaunchArgument("position_y", default_value="-1.0"),
        DeclareLaunchArgument("position_z", default_value="2.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value = odometry_source),
        gazebo,
        spawn_entity,
        rviz_node,
    ])