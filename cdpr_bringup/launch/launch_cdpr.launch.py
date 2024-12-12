import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("cdpr_bringup"), "config/",
                     "cdpr_rviz_config.rviz"]),
                     "-f", "/world"],
            on_exit=launch.actions.Shutdown()
        ),
        Node(
            package='cdpr_kinematics',
            executable='kinematics'
        ),
        Node(
            package='cdpr_driver',
            executable='driver'
        )
    ])