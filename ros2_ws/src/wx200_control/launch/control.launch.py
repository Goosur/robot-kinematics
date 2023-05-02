import os

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    wx200_control_share_path = FindPackageShare(
        "wx200_control").find("wx200_control")
    urdf_path = os.path.join(wx200_control_share_path, "urdf/wx200.urdf.xml")
    rviz_config_path = os.path.join(
        wx200_control_share_path, "rviz/urdf.rviz.yaml")

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    motor_writer = Node(
        package="wx200_control",
        executable="multi_motor_writer",
        name="wx200_control"
    )

    state_publisher = Node(
        package="wx200_control",
        executable="state_publisher",
        name="wx200_state_publisher",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path]
    )

    return LaunchDescription([
        # motor_writer,
        state_publisher,
        robot_state_publisher,
        rviz
    ])
