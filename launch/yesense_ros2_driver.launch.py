from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="yesense_ros2_driver",
            executable="yesense_ros2_driver_node",
            name="yesense_ros2_driver_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port": "/dev/imu"},
                {"baud_rate": 460800},
                {"frame_id": "imu_link"}
            ]
        )
    ])
