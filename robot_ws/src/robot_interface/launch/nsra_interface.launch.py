import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import xacro


def load_file(file_path):
    try:
        with open(file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print("file error")
        return None


def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print("yaml error")
        return None


def generate_launch_description():

    # Initialize Arguments
    runtime_config_package = "robot_descriptions" #LaunchConfiguration("runtime_config_package")
    runtime_config_robot = "nsra2" #LaunchConfiguration("runtime_config_robot")

    # planning_context
    robot_description_config = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [get_package_share_directory(runtime_config_package), runtime_config_robot, "urdf", "nsra2.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            "",
            " ",
            "use_sim:=",
            "false",
            " ",
            "use_fake_hardware:=",
            "false",
            " ",
            "fake_sensor_commands:=",
            "false",
            " ",
            "slowdown:=",
            "3.0",
        ]
    )
    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        os.path.join(
            get_package_share_directory(runtime_config_package),
            runtime_config_robot,
            "moveit",
            "nsra.srdf",
        )
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        os.path.join(
            get_package_share_directory(runtime_config_package),
            runtime_config_robot,
            "moveit",
            "kinematics.yaml",
        )
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="nsra2_interface",
        package="robot_interface",
        executable="nsra2_interface",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml],
    )

    return LaunchDescription([move_group_demo])