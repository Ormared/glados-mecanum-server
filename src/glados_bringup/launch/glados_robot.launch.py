from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():

    fox_bridge = LaunchConfiguration('fox_bridge', default='true')
    fox_bridge_dec = DeclareLaunchArgument(
        'fox_bridge',
        default_value='true',
        description='Run foxglove bridge'
    )

    pkg_project_description = get_package_share_directory('glados_description')
    pkg_project_hardware = get_package_share_directory('glados_hardware')
    urdf_file = os.path.join(pkg_project_description, 'models', 'glados', 'model.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    serial_params = os.path.join(pkg_project_hardware, 'config', 'params.yaml')
    teleop_params = os.path.join(pkg_project_hardware, 'config', 'teleop_params.yaml')

    serial = Node(
        package="glados_hardware",
        executable="serial_node",
        name="serial",
        parameters=[serial_params]
    )

    teleop = Node(
        package="glados_hardware",
        executable="teleop_to_serial_node",
        name="teleop_to_serial",
        parameters=[teleop_params]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {'use_sim_time': False},
            {'robot_description': robot_desc},
        ]
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        condition=IfCondition(fox_bridge),
        parameters=[
            {'robot_description': robot_desc},
        ]
    )

    return LaunchDescription([
        fox_bridge_dec,
        serial,
        teleop,
        robot_state_publisher,
        foxglove_bridge
    ])
