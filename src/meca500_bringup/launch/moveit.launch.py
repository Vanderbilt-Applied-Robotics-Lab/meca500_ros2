from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Arguments
    servo = LaunchConfiguration("servo")
    robot_controller = LaunchConfiguration("robot_controller")
    simulation = LaunchConfiguration("simulation")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    control_port = LaunchConfiguration("control_port")

    launch_arguments = {
        "servo": servo,
        "robot_controller": robot_controller,
        "simulation": simulation,
        "robot_ip": robot_ip,
        "robot_port": robot_port,
        "control_port": control_port
    }

    # MoveIt2
    moveit_config = (
        MoveItConfigsBuilder(
            "meca500",
            package_name="meca500_moveit"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # MoveIt2 Servo
    planning_group_name = {"planning_group_name": "meca500"}
    servo_params = {
        "moveit_servo": ParameterBuilder(
            "meca500_moveit"
        ).yaml("config/servo.yaml").to_dict()
    }

    # Controllers
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("mitsubishi_robot_bringup"),
         "config", "controllers.yaml"]
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("mitsubishi_robot_bringup"),
            "config",
            "rviz",
            "moveit.rviz",
        ]
    )

    # Nodes
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    joint_state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[robot_controller, "-c", "/controller_manager"],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
       	    moveit_config.planning_pipelines,
            moveit_config.planning_scene_monitor
        ],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.trajectory_execution,
            moveit_config.planning_pipelines,
        	moveit_config.planning_scene_monitor
        ],
        output="screen",
        condition=IfCondition(servo),
    )

    # Launch order
    nodes = [
        static_tf_node,
        control_node,
        joint_state_broadcaster_spawner_node,
        robot_controller_spawner_node,
        robot_state_publisher_node,
        move_group_node,
        servo_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_controller_spawner_node,
                on_exit=[rviz_node],
            ),
        ),
    ]

    return nodes


def generate_launch_description():
    # Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "servo",
            default_value="false",
            description="Launch MoveIt2 Servo",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Selected robot controller, options can be found in 'mitsubishi_robot_bringup/config/controllers.yaml'",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "simulation",
            default_value="10001",
            description="Run with simulation hardware",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.100",
            description="IP address of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port",
            default_value="10000",
            description="Port of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "control_port",
            default_value="10001",
            description="Port of the host computer",
        )
    )

    # Launch function
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
