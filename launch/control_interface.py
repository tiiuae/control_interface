import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
import sys

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "control_interface"
    pkg_share_path = get_package_share_directory(pkg_name)
    
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))
    ld.add_action(launch.actions.DeclareLaunchArgument("device_url", default_value="udp://:14590"))
    ld.add_action(launch.actions.DeclareLaunchArgument("world_frame", default_value="world"))

    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    dbg_sub = None
    if sys.stdout.isatty():
        dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'])

    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    namespace=DRONE_DEVICE_ID
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_control_interface',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='control_interface::ControlInterface',
                namespace=namespace,
                name='control_interface',
                parameters=[
                    pkg_share_path + '/config/control_interface.yaml',
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                    {"device_url": launch.substitutions.LaunchConfiguration("device_url")},
                    {"world_frame": launch.substitutions.LaunchConfiguration("world_frame")},
                ],
                remappings=[
                    ("~/diagnostics_out", "~/diagnostics"),
                    ("~/waypoints_out", "~/waypoints"),

                    ("~/octomap_reset_out", "/" + DRONE_DEVICE_ID + "/octomap_server/reset"),

                    ("~/control_mode_in", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_control_mode/out"),
                    ("~/land_detected_in", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_land_detected/out"),
                    ("~/mission_result_in", "/" + DRONE_DEVICE_ID + "/fmu/mission_result/out"),
                    ("~/home_position_in", "/" + DRONE_DEVICE_ID + "/fmu/home_position/out"),
                    ("~/local_odom_in", "/" + DRONE_DEVICE_ID + "/odometry/local_odom"),
                    
                    ("~/arming_in", "~/arming"),
                    ("~/takeoff_in", "~/takeoff"),
                    ("~/land_in", "~/land"),
                    ("~/local_waypoint_in", "~/local_waypoint"),
                    ("~/local_path_in", "~/local_path"),
                    ("~/gps_waypoint_in", "~/gps_waypoint"),
                    ("~/gps_path_in", "~/gps_path"),
                    
                    ("~/waypoint_to_local_in", "~/waypoint_to_local"),
                    ("~/path_to_local_in", "~/path_to_local"),

                    ("~/get_px4_param_int_in", "~/get_px4_param_int"),
                    ("~/set_px4_param_int_in", "~/set_px4_param_int"),
                    ("~/get_px4_param_float_in", "~/get_px4_param_float"),
                    ("~/set_px4_param_float_in", "~/set_px4_param_float"),

                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))

    return ld
