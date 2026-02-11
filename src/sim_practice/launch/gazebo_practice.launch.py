from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("sim_practice")
    urdf_path = os.path.join(pkg_share, "urdf", "sim_practice.urdf.xml")
    rviz_config_path = os.path.join(pkg_share, "urdf_sim_config.rviz")
    
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    # Optional: helpful for RViz sliders; not required for Gazebo physics
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "practice_rover",
            "-topic", "robot_description",
            # Spawn height; use this instead of a URDF "world" link
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.2",
        ],
        output="screen",
    )

    spawn_delayed = TimerAction(
        period=2.0,
        actions=[spawn_node]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        gazebo_launch,
        rsp_node,
        jsp_gui_node,
        spawn_delayed,
        rviz_node,
    ])