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
                    {"device_url": launch.substitutions.LaunchConfiguration("device_url")},
                ],
                remappings=[
                    ("~/desired_pose_out", "~/desired_pose"),
                    ("~/diagnostics_out", "~/diagnostics"),
                    ("~/waypoint_markers_out", "~/waypoint_markers"),

                    ("~/octomap_reset_out", "/" + DRONE_DEVICE_ID + "/octomap_server/reset"),

                    ("~/diagnostics_out", "~/diagnostics"),
                    ("~/control_mode_in", "/" + DRONE_DEVICE_ID + "/VehicleControlMode_PubSubTopic"),
                    ("~/land_detected_in", "/" + DRONE_DEVICE_ID + "/VehicleLandDetected_PubSubTopic"),
                    ("~/mission_result_in", "/" + DRONE_DEVICE_ID + "/MissionResult_PubSubTopic"),
                    ("~/home_position_in", "/" + DRONE_DEVICE_ID + "/HomePosition_PubSubTopic"),
                    
                    ("~/arming_in", "~/arming"),
                    ("~/takeoff_in", "~/takeoff"),
                    ("~/land_in", "~/land"),
                    ("~/local_waypoint_in", "~/local_waypoint"),
                    ("~/local_path_in", "~/local_path"),
                    ("~/gps_waypoint_in", "~/gps_waypoint"),
                    ("~/gps_path_in", "~/gps_path"),
                    ("~/local_odom_in", "/" + DRONE_DEVICE_ID + "/odometry2/local_odom"),
                    
                    ("~/waypoint_to_local_in", "~/waypoint_to_local"),
                    ("~/path_to_local_in", "~/path_to_local"),

                    ("~/get_origin", "/" + DRONE_DEVICE_ID + "/odometry2/get_origin"),
                    ("~/getting_odom", "/" + DRONE_DEVICE_ID + "/odometry2/getting_odom"),

                    ("~/get_px4_param_int", "~/get_px4_param_int"),
                    ("~/set_px4_param_int", "~/set_px4_param_int"),
                    ("~/set_px4_param_float", "~/set_px4_param_float"),

                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))

    return ld
