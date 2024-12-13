import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    my_package_name='glados_bringup'
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_dec = DeclareLaunchArgument('use_sim_time', default_value='false')

    tracker_params_sim = os.path.join(get_package_share_directory(my_package_name),'config','ball_tracker_params_sim.yaml')
    tracker_params_robot = os.path.join(get_package_share_directory(my_package_name),'config','ball_tracker_params_robot.yaml')

    params_path = PythonExpression(['"',tracker_params_sim, '" if "true" == "', use_sim_time, '" else "', tracker_params_robot, '"'])

    tracker_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ball_tracker'), 'launch', 'ball_tracker.launch.py')]),
                    launch_arguments={'params_file': params_path,
                                    'image_topic': '/camera/image',
                                    'cmd_vel_topic': '/cmd_vel',
                                    'enable_3d_tracker': 'true'}.items())

    return LaunchDescription([
        use_sim_time_dec,
        tracker_launch,
    ])
