from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="serial_communication",
            executable="serial_node",
            name="serial_node",
            parameters=["src/serial_communication/config/params.yaml"]
        ),
        Node(
            package="teleop_to_serial",
            executable="teleop_to_serial_node",
            name="teleop_to_serial"
        )
    ])