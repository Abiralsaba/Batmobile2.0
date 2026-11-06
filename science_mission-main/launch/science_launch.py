from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="science_mission",
            executable="camera_feed_node",
            name="camera_feed_node",
            # parameters=[{"camera_id": "ROG", "publish_rate": 30.0}],
        ),
        Node(
            package="science_mission",
            executable="arm_node",
            name="arm_node",
            # parameters=[{"teensy_serial": "16832890", "serial_port": "/dev/ttyACM0", "baud_rate": 115200}],
        ),
        Node(
            package="science_mission",
            executable="bifrost_node",
            name="bifrost_node",
            # parameters=[{"teensy_serial": "16832280", "serial_port": "/dev/ttyACM1", "baud_rate": 115200}],
        ),
        Node(
            package="science_mission",
            executable="sensor_fusion_node",
            name="sensor_fusion_node",
            # parameters=[{"nano_vid": "6790", "serial_port": "/dev/ttyUSB0", "baud_rate": 9600, "publish_rate": 10.0}],
        ),
        Node(
            package="science_mission",
            executable="numpad_publisher",
            name="numpad_publisher",
        ),

    ])
