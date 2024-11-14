from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_project_bringup = get_package_share_directory('glados_bringup')
    pkg_project_description = get_package_share_directory('glados_description')

    # Load the SDF file from "description" package
    sdf_file = os.path.join(pkg_project_description, 'models', 'glados', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'rviz.rviz')],
    )

    return LaunchDescription([
        Node(
            package="glados_hardware",
            executable="serial_node",
            name="serial_node",
            parameters=["src/glados_hardware/config/params.yaml"]
        ),
        Node(
            package="glados_hardware",
            executable="teleop_to_serial_node",
            name="teleop_to_serial"
        ),
        Node(
            package="glados_application",
            executable="joint_state_publisher_node",
            name="joint_state_publisher"
        ),
        Node(
            package="glados_application",
            executable="odometry_node",
            name="odometry"
        ),
        robot_state_publisher,
        rviz
    ])