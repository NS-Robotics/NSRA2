import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import xacro #sudo apt install ros-foxy-xacro


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

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(
        os.path.join(
            get_package_share_directory(runtime_config_package),
            runtime_config_robot,
            "moveit",
            "ompl_planning.yaml",
        )
    )
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        os.path.join(
            get_package_share_directory(runtime_config_package),
            runtime_config_robot,
            "config",
            "moveit_controllers.yaml",
        )
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    joint_limits_yaml = {
        "robot_description_planning": load_yaml(
            os.path.join(
                get_package_share_directory(runtime_config_package),
                runtime_config_robot,
                "moveit",
                "joint_limits.yaml",
            )
        )
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
        ],
    )

    # RViz
    # rviz_config_file = PathJoinSubstitution(
    #     [
    #         FindPackageShare(runtime_config_package), 
    #         runtime_config_robot, 
    #         "config", 
    #         "robot.rviz"
    #     ]
    # )

    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     parameters=[
    #         robot_description,
    #         robot_description_semantic,
    #         ompl_planning_pipeline_config,
    #         kinematics_yaml,
    #         joint_limits_yaml,
    #     ],
    # )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Warehouse mongodb server
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "192.168.1.110"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            #rviz_node,
            static_tf,
            #robot_state_publisher,
            run_move_group_node,
            #ros2_control_node,
            mongodb_server_node,
        ]
        #+ load_controllers
    )