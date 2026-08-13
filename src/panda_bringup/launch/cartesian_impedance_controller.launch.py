from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    description_file = PathJoinSubstitution([FindPackageShare('franka_description'), 'robots', 'panda_arm.urdf.xacro'])
    rviz_file = PathJoinSubstitution([FindPackageShare('franka_description'), 'rviz', 'visualize_franka.rviz'])
    controller_config = PathJoinSubstitution([FindPackageShare('panda_bringup'), 'config', 'cartesian_impedance_controller.yaml'])
    robot_description = Command([FindExecutable(name='xacro'), ' ', description_file,
        ' hand:=', LaunchConfiguration('load_gripper'), ' use_fake_hardware:=true'])
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip'),
        DeclareLaunchArgument('operator_confirm', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('load_gripper', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),
        Node(package='panda_bringup', executable='cartesian_impedance_controller', output='screen',
             parameters=[controller_config, {'robot_ip': LaunchConfiguration('robot_ip'),
                                             'operator_confirm': LaunchConfiguration('operator_confirm')}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
