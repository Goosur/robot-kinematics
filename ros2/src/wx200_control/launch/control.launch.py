import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.substitutions import Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    wx200_control_path = get_package_share_path("wx200_control")
    urdf_path = os.path.join(wx200_control_path, "urdf/wx200.urdf.xml")
    rviz_config_path = os.path.join(
        wx200_control_path, "rviz/urdf.rviz.yaml")

    robot_description = ParameterValue(
        Command(["cat ", urdf_path]), value_type=str)

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

    world_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=['--frame-id "world"', '--child-frame-id "base_link"']
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path]
    )

    return LaunchDescription([
        motor_writer,
        state_publisher,
        world_publisher,
        robot_state_publisher,
        rviz
    ])
