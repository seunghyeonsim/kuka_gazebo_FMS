from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    apriltag_ros_prefix = get_package_share_directory('apriltag_ros')
    tags_36h11_yaml = os.path.join(apriltag_ros_prefix, 'cfg', 'tags_36h11.yaml')

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                ('/image_rect', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info')
            ],
            parameters=[
                {'use_sim_time': True},
                tags_36h11_yaml
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
