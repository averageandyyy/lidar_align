from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _str_to_bool(value: str) -> bool:
    value = value.strip().lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"Invalid boolean value: {value}")


def _create_node(context, *args, **kwargs):
    package_name = "lidar_align"

    config_file = LaunchConfiguration("config_file").perform(context)
    input_bag_path = LaunchConfiguration("input_bag_path").perform(context).strip()
    input_csv_path = LaunchConfiguration("input_csv_path").perform(context).strip()
    transforms_from_csv = LaunchConfiguration("transforms_from_csv").perform(context).strip()

    parameters = [config_file]

    overrides = {}

    # 指定されたときだけ上書きする。未指定なら YAML をそのまま使う。
    if input_bag_path:
        overrides["input_bag_path"] = input_bag_path

    if input_csv_path:
        overrides["input_csv_path"] = input_csv_path

    if transforms_from_csv:
        overrides["transforms_from_csv"] = _str_to_bool(transforms_from_csv)

    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package=package_name,
            executable="lidar_align_node",
            name="lidar_align",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description():
    package_name = "lidar_align"

    default_config = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "lidar_align.param.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to parameter YAML file",
        ),
        DeclareLaunchArgument(
            "input_bag_path",
            default_value="",
            description="Override input_bag_path only when non-empty",
        ),
        DeclareLaunchArgument(
            "input_csv_path",
            default_value="",
            description="Override input_csv_path only when non-empty",
        ),
        DeclareLaunchArgument(
            "transforms_from_csv",
            default_value="",
            description="Override transforms_from_csv only when set to true/false",
        ),
        OpaqueFunction(function=_create_node),
    ])

"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "lidar_align"

    default_config = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "lidar_align.param.yaml"]
    )

    default_bag = ""
    default_csv = ""

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to parameter YAML file",
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="",
        description="Override pointcloud_topic only when non-empty",
    )

    bag_arg = DeclareLaunchArgument(
        "input_bag_path",
        default_value=default_bag,
        description="Path to rosbag2 directory or file",
    )

    csv_arg = DeclareLaunchArgument(
        "input_csv_path",
        default_value=default_csv,
        description="Path to Maplab-format CSV file",
    )

    transforms_from_csv_arg = DeclareLaunchArgument(
        "transforms_from_csv",
        default_value="false",
        description="Load transforms from CSV instead of rosbag",
    )

    node = Node(
        package=package_name,
        executable="lidar_align_node",
        name="lidar_align",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "input_bag_path": LaunchConfiguration("input_bag_path"),
                "input_csv_path": LaunchConfiguration("input_csv_path"),
                "transforms_from_csv": LaunchConfiguration("transforms_from_csv"),
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
            },
        ],
    )

    return LaunchDescription(
        [
            config_arg,
            bag_arg,
            csv_arg,
            transforms_from_csv_arg,
            pointcloud_topic_arg,
            node,
        ]
    )"""