from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("realsense2_camera"), '/launch',
                 '/rs_launch.py']
            ),
            launch_arguments=[['depth.enable', 'false']]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0.0', '0.0', '-0.053', '0.0', '0.0', '0.0', '1.0', 'ee', 'ee_center']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0.017', '0.02', '1.955', '0.0', '0.7071', '0.0', '0.7071', 'world', 'camera_link']
        ),
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[('/image_rect/compressed','/camera/camera/color/image_raw/compressed'),
                        ('/camera_info','/camera/camera/color/camera_info')
                        ],
            parameters=[[FindPackageShare("cdpr_bringup"), '/config',
                 '/tags_36h11.yaml']]
        )

    ])