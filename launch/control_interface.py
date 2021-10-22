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
                ],
                remappings=[
                    ("~/vehicle_command_out", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_command/in"),
                    ("~/local_odom_out", "~/local_odom"),
                    ("~/diagnostics_out", "~/diagnostics"),
                    ("~/debug_markers_out", "~/debug/waypoint_markers"),

                    ("~/octomap_reset_out", "/" + DRONE_DEVICE_ID + "/octomap_server/reset"),

                    ("~/gps_in", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_global_position/out"),
                    ("~/pixhawk_odom_in", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_odometry/out"),
                    ("~/control_mode_in", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_control_mode/out"),
                    ("~/land_detected_in", "/" + DRONE_DEVICE_ID + "/fmu/vehicle_land_detected/out"),
                    ("~/mission_result_in", "/" + DRONE_DEVICE_ID + "/fmu/mission_result/out"),

                    ("~/arming_in", "~/arming"),
                    ("~/takeoff_in", "~/takeoff"),
                    ("~/land_in", "~/land"),
                    ("~/local_waypoint_in", "~/local_waypoint"),
                    ("~/local_path_in", "~/local_path"),
                    ("~/gps_waypoint_in", "~/gps_waypoint"),
                    ("~/gps_path_in", "~/gps_path"),
                    
                    ("~/waypoint_to_local_in", "~/waypoint_to_local"),
                    ("~/path_to_local_in", "~/path_to_local"),
                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))

    return ld
