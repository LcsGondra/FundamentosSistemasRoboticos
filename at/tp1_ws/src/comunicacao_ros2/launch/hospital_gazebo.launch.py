import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("comunicacao_ros2")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    urdf_file = os.path.join(pkg_share, "urdf", "robo_hospitalar.urdf")
    with open(urdf_file, "r") as infp:
        robot_desc = infp.read()

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Set to true to run Gazebo GUI client",
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        )
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": True}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "robo_hospitalar", "-topic", "robot_description", "-z", "0.10"],
        output="screen",
    )

    monitor_node = Node(
        package="comunicacao_ros2",
        executable="monitor_transporte",
        name="monitor_transporte",
        output="screen",
        parameters=[{"temperatura_limite": 27.0}],
    )

    echo_alertas = ExecuteProcess(
        cmd=["ros2", "topic", "echo", "/alertas"],
        output="screen",
    )

    return LaunchDescription(
        [
            gui_arg,
            gzserver,
            gzclient,
            rsp_node,
            spawn_entity,
            monitor_node,
            echo_alertas,
        ]
    )
