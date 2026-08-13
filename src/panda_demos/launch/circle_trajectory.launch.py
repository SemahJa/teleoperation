from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([Node(package='panda_demos', executable='cartesian_circle_publisher', output='screen')])
