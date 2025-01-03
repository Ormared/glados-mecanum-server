from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_project_bringup = get_package_share_directory('glados_bringup')
    pkg_project_description = get_package_share_directory('glados_description')

    sdf_file = os.path.join(pkg_project_description, 'models', 'glados', 'model.urdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc}
        ]
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    

    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'rviz.rviz')]
    )

    return LaunchDescription([
        rsp,
        jsp,
        rviz
    ])
