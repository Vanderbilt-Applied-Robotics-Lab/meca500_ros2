from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # --------------------------
    # Launch arguments
    # --------------------------
    servo = LaunchConfiguration("servo")
    robot_controller = LaunchConfiguration("robot_controller")
    simulation = LaunchConfiguration("simulation")
    robot_ip = LaunchConfiguration("robot_ip")
    robot_port = LaunchConfiguration("robot_port")
    control_port = LaunchConfiguration("control_port")
    hardware_type = LaunchConfiguration("hardware_type")

    # --------------------------
    # MoveIt configuration
    # --------------------------
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="meca500",
            package_name="meca500_moveit",
        )
        .robot_description(
            mappings={
                "hardware_type": hardware_type,
                "simulation": simulation,
                "robot_ip": robot_ip,
                "robot_port": robot_port,
                "control_port": control_port,
            }
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    # --------------------------
    # MoveIt Servo parameters
    # --------------------------
    servo_params = {
        "moveit_servo": ParameterBuilder("meca500_moveit")
        .yaml("config/servo.yaml")
        .to_dict()
    }

    planning_group_name = {"planning_group_name": "meca500"}

    # --------------------------
    # Controller config
    # --------------------------
    controllers_yaml = PathJoinSubstitution(
        [
            FindPackageShare("meca500_bringup"),
            "config",
            "controllers.yaml",
        ]
    )

    # --------------------------
    # RViz config
    # --------------------------
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("meca500_bringup"),
            "config",
            "rviz",
            "moveit.rviz",
        ]
    )

    # --------------------------
    # Nodes
    # --------------------------
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        arguments=["--frame-id", "world", "--child-frame-id", "meca_base_link"],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            controllers_yaml,
            moveit_config.robot_description,
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot_controller,
            "--controller-manager",
            "/controller_manager",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
        ],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        condition=IfCondition(servo),
        parameters=[
            servo_params,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_scene_monitor,
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        arguments=["-d", rviz_config],
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
        ],
    )

    # --------------------------
    # Launch ordering
    # --------------------------
    return [
        static_tf_node,
        ros2_control_node,
        joint_state_broadcaster,
        robot_controller_spawner,
        robot_state_publisher,
        move_group,
        servo_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=robot_controller_spawner,
                on_exit=[rviz],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "servo",
                default_value="false",
                description="Launch MoveIt Servo",
            ),
            DeclareLaunchArgument(
                "robot_controller",
                default_value="joint_trajectory_controller",
                description="ros2_control controller to use",
            ),
            DeclareLaunchArgument(
                "simulation",
                default_value="false",
                description="Run with simulated hardware",
            ),
            DeclareLaunchArgument(
                "robot_ip",
                default_value="192.168.0.100",
                description="Meca500 IP address",
            ),
            DeclareLaunchArgument(
                "robot_port",
                default_value="10000",
                description="Robot command port",
            ),
            DeclareLaunchArgument(
                "control_port",
                default_value="10001",
                description="Host control port",
            ),
            DeclareLaunchArgument(
                'hardware_type',
                default_value='meca500_hardware',
                description='Hardware plugin to load'
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
