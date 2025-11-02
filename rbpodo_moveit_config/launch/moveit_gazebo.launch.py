from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EnvironmentVariable,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.parameter_descriptions import ParameterValue

"""
Unified Gazebo + MoveIt launch
• Spawns robot into Gazebo from /robot_description (built from Xacro)
• Uses gz_ros2_control inside Gazebo (do NOT launch external ros2_control_node)
• Spawns controllers AFTER Gazebo is ready
• Starts MoveIt (move_group) + RViz using the SAME description

Notes:
- "world"와 "rviz_config"는 기본 경로를 제공하므로 실행 시 인자를 생략해도 동작합니다.
- 필요 시 model_id만 바꾸면 동일한 파이프라인 유지.
"""

def generate_launch_description():
    # ── Launch arguments (with safe defaults so the user doesn't have to pass them)
    model_id = LaunchConfiguration("model_id")
    world = LaunchConfiguration("world")
    rviz_config = LaunchConfiguration("rviz_config")

    # Package shares
    rb_desc = FindPackageShare("rbpodo_description")
    rb_moveit = FindPackageShare("rbpodo_moveit_config")

    # Defaults (no need to pass from CLI)
    default_world = PathJoinSubstitution([rb_moveit, "worlds", "playground.sdf"])  # adjust if your world differs
    default_rviz = PathJoinSubstitution([rb_moveit, "config", "moveit.rviz"])     # falls back to RViz default if missing

    # Gazebo (ros_gz_sim) entrypoint
    gz_sim_launch = PathJoinSubstitution([
        FindPackageShare("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    ])

    # Build robot_description once (Xacro → URDF) and share with Gazebo + MoveIt
    xacro_file = PathJoinSubstitution([rb_moveit, "config", "rbpodo.urdf.xacro"])  # keep single source of truth
    # 2) robot_description 생성부에서 --inorder 삭제하고 --verbosity 0 추가
    robot_description_param = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ", xacro_file,
                " ", "model_id:=", model_id,
                " ", "use_gazebo:=true",
            ]),
            value_type=str,  # ← 문자열 강제 (YAML 파싱 방지)
        )
    }

    # MoveIt configuration derived from your existing package
    moveit_cfg = (
        MoveItConfigsBuilder("rbpodo")
        .robot_description(file_path="config/rbpodo.urdf.xacro", mappings={"model_id": model_id})
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl"])  # extend if you use pilz/chomp
        .to_moveit_configs()
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────
    # 1) Gazebo (server/client via ros_gz_sim standard launcher)
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={"gz_args": ["-r ", world]}.items(),  # -r to run immediately
    )

    # 2) robot_state_publisher: publishes TF from /robot_description
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_param, {"use_sim_time": True}],
    )

    # 3) Spawn robot into Gazebo from the /robot_description topic
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", model_id, "-topic", "robot_description", "-x", "1.0","-z", "0.0", ],
    )

    delayed_spawn_robot = TimerAction(period=5.0, actions=[spawn])


    # 4) Bridge /clock (Gazebo → ROS2)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # 5) MoveIt move_group
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_cfg.to_dict(), {"use_sim_time": True}],
    )

    # 6) RViz (loads MoveIt displays; if rviz_config points to a missing file, RViz uses default layout)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_cfg.robot_description,
            moveit_cfg.robot_description_semantic,
            moveit_cfg.robot_description_kinematics,
            moveit_cfg.planning_pipelines,
            moveit_cfg.joint_limits,
            {"use_sim_time": True},
        ],
    )

    # 7) Controller spawners (use Gazebo-embedded controller_manager)
    spawn_js = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
    )
    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=[
            # "/rgb_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            # "/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            # "/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_rgb@sensor_msgs/msg/Image@gz.msgs.Image",
            # '/camera_rgb_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            
        ],
        parameters=[{"use_sim_time": True}],
    )

    # Delay spawners until Gazebo + gz_ros2_control are alive
    delayed_spawn_js = TimerAction(period=5.0, actions=[spawn_js])
    delayed_spawn_arm = TimerAction(period=6.5, actions=[spawn_arm])

    # Environment: ensure model://rbpodo_description/... resolves
    desc_share = PathJoinSubstitution([rb_desc])
    desc_share_parent = PathJoinSubstitution([desc_share, ".."])  # …/share

    set_paths = [
        SetEnvironmentVariable(
            name="IGN_GAZEBO_RESOURCE_PATH",
            value=[desc_share_parent, ":", EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value="")],
        ),
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=[desc_share_parent, ":", EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")],
        ),
        SetEnvironmentVariable(
            name="IGN_FILE_PATH",
            value=[desc_share_parent, ":", EnvironmentVariable("IGN_FILE_PATH", default_value="")],
        ),
        SetEnvironmentVariable(
            name="GZ_FILE_PATH",
            value=[desc_share_parent, ":", EnvironmentVariable("GZ_FILE_PATH", default_value="")],
        ),
    ]

    # Declarations with defaults so the user doesn't need to pass them
    declared = [
        DeclareLaunchArgument("model_id", default_value="rb5_farmily"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
    ]

    return LaunchDescription(
        declared
        + set_paths
        + [
            gz,
            rsp,
            delayed_spawn_robot,
            clock_bridge,
            move_group,
            rviz,
            delayed_spawn_js,
            delayed_spawn_arm,
            camera_bridge,
        ]
    )