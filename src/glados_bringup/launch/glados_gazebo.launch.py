# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    fox_bridge = LaunchConfiguration('fox_bridge', default='true')
    fox_bridge_dec = DeclareLaunchArgument(
        'fox_bridge',
        default_value='true',
        description='Run foxglove bridge'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world = LaunchConfiguration('world', default='cubes_with_ball.sdf')
    # world = LaunchConfiguration('world', default='perimeter.sdf')
    # world = LaunchConfiguration('world', default='warehouse.sdf')
    # world = LaunchConfiguration('world', default='glados_obs.sdf')

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('glados_bringup')
    pkg_project_gazebo = get_package_share_directory('glados_gazebo')
    pkg_project_description = get_package_share_directory('glados_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    ekf_config_path = PathJoinSubstitution([pkg_project_bringup, 'config', 'ekf.yaml'])

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'glados', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PathJoinSubstitution([
                pkg_project_gazebo,
                'worlds',
                world
            ]),
            'use_sim_time': use_sim_time,
            # 'publish_joints': 'false',
        }.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'rviz.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz')),
       parameters=[{'use_sim_time': use_sim_time}]
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        condition=IfCondition(fox_bridge),
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'glados_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    pkg_project_hardware = get_package_share_directory('glados_hardware')
    teleop_params = os.path.join(pkg_project_hardware, 'config', 'teleop_params.yaml')

    # Teleop
    teleop = Node(
        package='glados_hardware',
        executable='teleop_to_serial_node',
        name='teleop',
        parameters=[
            teleop_params,
            {"use_sim_time": True}
        ]
    )

    # Serial gz adapter
    serial_gz_adapter = Node(
        package='glados_hardware',
        executable='serial_gz_adapter_node',
        name='serial_gz_adapter',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot Localization EKF node
    ekf_robot_localization = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}, 
                ekf_config_path
            ],
            # remappings=[("odometry/filtered", "odom")]
    )

    return LaunchDescription([
        declare_use_sim_time,
        bridge,
        gz_sim,
        serial_gz_adapter,
        teleop,
        robot_state_publisher,
        ekf_robot_localization,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        rviz,
        foxglove,
    ])