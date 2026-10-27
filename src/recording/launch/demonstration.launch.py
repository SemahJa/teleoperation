"""Camera + Quest + Menu recording, using the existing robot-free composition."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    def include(package,file,args):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory(package))/'launch'/file)),launch_arguments=args.items())
    arg=LaunchConfiguration
    defaults={'output_root':'/tmp/franka_demonstrations','cameras':'both','wrist_profile':'target','scene_profile':'target','listen_port':'45454',
              'arms_config':str(Path(get_package_share_directory('quest_teleop'))/'config'/'arms.yaml')}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[
        include('camera_bringup','cameras.launch.py',{'cameras':arg('cameras'),'wrist_profile':arg('wrist_profile'),'scene_profile':arg('scene_profile')}),
        include('teleop_bringup','robot_free.launch.py',{'arms_config':arg('arms_config'),
            'start_bridge':'true','mock_measured_pose':'true','listen_port':arg('listen_port'),
            'recording_enabled':'false'}),
        include('recording','recording.launch.py',{'arms_config':arg('arms_config'),'recording_mode':'menu',
            'recording_config':str(Path(get_package_share_directory('recording'))/'config'/'demonstration.yaml'),
            'recording_output_root':arg('output_root'),'mock_measured_pose':'true'})])
