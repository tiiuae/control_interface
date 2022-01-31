/* includes //{ */

#include <connection_result.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <fog_msgs/msg/control_interface_diagnostics.hpp>
#include <fog_msgs/srv/path.hpp>
#include <fog_msgs/srv/path_to_local.hpp>
#include <fog_msgs/srv/get_px4_param_int.hpp>
#include <fog_msgs/srv/get_px4_param_float.hpp>
#include <fog_msgs/srv/set_px4_param_int.hpp>
#include <fog_msgs/srv/set_px4_param_float.hpp>
#include <fog_msgs/srv/vec4.hpp>
#include <fog_msgs/srv/waypoint_to_local.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mavsdk/geometry.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/log_callback.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <memory>
#include <mutex>
#include <fstream>
#include <boost/circular_buffer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/mission_result.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'
#include <unordered_map>

#include <fog_lib/params.h>
#include <fog_lib/mutex_utils.h>
#include <fog_lib/scope_timer.h>
#include <fog_lib/misc.h>
#include <fog_lib/geometry/misc.h>

#include "control_interface/utils.h"
#include "control_interface/enums.h"
#include "control_interface/mission_manager.h"

//}

using namespace std::placeholders;
using namespace fog_lib;
using namespace fog_lib::geometry;

namespace control_interface
{

struct local_waypoint_t
{
  double x;
  double y;
  double z;
  double heading;
};

struct gps_waypoint_t
{
  double latitude;
  double longitude;
  double altitude;
  double heading;
};

/* coordinate system conversions //{ */

/* globalToLocal //{ */
std::pair<double, double> globalToLocal(const std::shared_ptr<mavsdk::geometry::CoordinateTransformation> &coord_transform, const double &latitude_deg,
                                        const double &longitude_deg) {
  mavsdk::geometry::CoordinateTransformation::GlobalCoordinate global;
  global.latitude_deg  = latitude_deg;
  global.longitude_deg = longitude_deg;
  auto local           = coord_transform->local_from_global(global);

  return {local.east_m, local.north_m};
}

local_waypoint_t globalToLocal(const std::shared_ptr<mavsdk::geometry::CoordinateTransformation> &coord_transform, const gps_waypoint_t &wg) {
  local_waypoint_t                                             wl;
  mavsdk::geometry::CoordinateTransformation::GlobalCoordinate global;
  global.latitude_deg  = wg.latitude;
  global.longitude_deg = wg.longitude;
  auto local           = coord_transform->local_from_global(global);

  wl.x   = local.east_m;
  wl.y   = local.north_m;
  wl.z   = wg.altitude;
  wl.heading = wg.heading;
  return wl;
}

std::vector<local_waypoint_t> globalToLocal(const std::shared_ptr<mavsdk::geometry::CoordinateTransformation> &coord_transform,
                                            const std::vector<gps_waypoint_t> &                                wgs) {
  std::vector<local_waypoint_t> wls;
  for (const auto &wg : wgs) {
    wls.push_back(globalToLocal(coord_transform, wg));
  }
  return wls;
}
//}

/* localToGlobal //{ */
std::pair<double, double> localToGlobal(const std::shared_ptr<mavsdk::geometry::CoordinateTransformation> &coord_transform, const double &x, const double &y) {
  mavsdk::geometry::CoordinateTransformation::LocalCoordinate local;
  local.north_m = y;
  local.east_m  = x;
  auto global   = coord_transform->global_from_local(local);
  return {global.latitude_deg, global.longitude_deg};
}

gps_waypoint_t localToGlobal(const std::shared_ptr<mavsdk::geometry::CoordinateTransformation> &coord_transform, const local_waypoint_t &wl) {
  gps_waypoint_t                                              wg;
  mavsdk::geometry::CoordinateTransformation::LocalCoordinate local;
  local.north_m = wl.y;
  local.east_m  = wl.x;
  auto global   = coord_transform->global_from_local(local);
  wg.latitude   = global.latitude_deg;
  wg.longitude  = global.longitude_deg;
  wg.altitude   = wl.z;
  wg.heading        = wl.heading;
  return wg;
}

std::vector<gps_waypoint_t> localToGlobal(const std::shared_ptr<mavsdk::geometry::CoordinateTransformation> &coord_transform,
                                          const std::vector<local_waypoint_t> &                              wls) {
  std::vector<gps_waypoint_t> wgs;
  for (const auto &wl : wls) {
    wgs.push_back(localToGlobal(coord_transform, wl));
  }
  return wgs;
}
//}

//}

/* add_unhealthy_reasons helper string function //{ */

void add_unhealthy_reasons(const mavsdk::Telemetry::Health& health, std::string& to_str)
{
  add_reason_if("gyrometer calibration not ok", !health.is_gyrometer_calibration_ok, to_str);
  add_reason_if("accelerometer calibration not ok", !health.is_accelerometer_calibration_ok, to_str);
  add_reason_if("magnetometer calibration not ok", !health.is_magnetometer_calibration_ok, to_str);
  add_reason_if("local position not ok", !health.is_local_position_ok, to_str);
  add_reason_if("global position not ok", !health.is_global_position_ok, to_str);
  add_reason_if("home position not ok", !health.is_home_position_ok, to_str);
  add_reason_if("not armable", !health.is_armable, to_str);
}

//}

/* is_healthy() method //{ */
bool is_healthy(const mavsdk::Telemetry::Health& health)
{
  return health.is_gyrometer_calibration_ok && health.is_accelerometer_calibration_ok && health.is_magnetometer_calibration_ok && health.is_local_position_ok && health.is_global_position_ok && health.is_home_position_ok && health.is_armable;
}
//}

// --------------------------------------------------------------
// |             ControlInterface class declaration             |
// --------------------------------------------------------------

/* class ControlInterface //{ */
class ControlInterface : public rclcpp::Node
{
public:
  ControlInterface(rclcpp::NodeOptions options);

private:
  std::recursive_mutex state_mutex_;
  vehicle_state_t vehicle_state_ = vehicle_state_t::not_connected;
  std::string vehicle_state_str_ = "MavSDK system not connected";

  std::atomic_bool system_connected_ = false; // set to true when MavSDK connection is established
  std::atomic_bool getting_control_mode_ = false; // set to true when a VehicleControlMode is received from pixhawk
  std::atomic_bool getting_odom_ = false;
  std::atomic_bool gps_origin_set_ = false;

  std::atomic_bool manual_override_ = false;

  std::unique_ptr<MissionManager> mission_mgr_ = nullptr;

  // the mavsdk::Action is used for requesting actions such as takeoff and landing
  std::mutex action_mutex_;
  std::shared_ptr<mavsdk::Action> action_;

  // the mavsdk::Param is used for setting and reading PX4 parameters
  std::recursive_mutex param_mutex_;
  std::shared_ptr<mavsdk::Param> param_;

  // the mavsdk::Telemetry is used for reading PX4 telemetry data (e.g. battery, GPS, RC connection, flight mode etc.)
  std::recursive_mutex telem_mutex_;
  std::shared_ptr<mavsdk::Telemetry> telem_;

  std::string uav_name_    = "";
  std::string world_frame_ = "";

  std::string                      device_url_;
  mavsdk::Mavsdk                   mavsdk_;
  std::shared_ptr<mavsdk::System>  system_;


  // use takeoff lat and long to initialize local frame
  std::mutex coord_transform_mutex_;
  std::shared_ptr<mavsdk::geometry::CoordinateTransformation> coord_transform_;
  Eigen::Vector3d home_position_offset_ = Eigen::Vector3d(0, 0, 0); // this is also protected by the coord_transform_mutex_

  // vehicle local position
  std::mutex pose_mutex_;
  boost::circular_buffer<Eigen::Vector3d> pose_takeoff_samples_;
  Eigen::Vector3d              pose_pos_;
  tf2::Quaternion              pose_ori_;

  std::mutex mavsdk_logging_mutex_;
  std::ofstream mavsdk_logging_file_;

  rclcpp::Duration print_callback_throttle_ = rclcpp::Duration(std::chrono::seconds(1));
  // config params
  bool   print_callback_durations_          = false;
  rclcpp::Duration print_callback_min_dur_  = rclcpp::Duration::from_seconds(0.020);
  int    mavsdk_logging_print_level_        = 1;
  std::string mavsdk_logging_filename_      = {};
  double heading_offset_correction_         = M_PI / 2;
  double takeoff_height_                    = 2.5;
  double control_update_rate_               = 10.0;
  double diagnostics_publish_rate_          = 1.0;
  double waypoint_loiter_time_              = 0.0;
  bool   octomap_reset_before_takeoff_      = true;
  double octomap_reset_timeout_             = 3.0;
  float  waypoint_acceptance_radius_        = 0.3f;
  float  altitude_acceptance_radius_        = 0.2f;
  double target_velocity_                   = 1.0;
  int    takeoff_position_samples_          = 20;
  int    mission_upload_attempts_threshold_ = 5;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr               waypoint_publisher_;
  rclcpp::Publisher<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr  diagnostics_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr             cmd_pose_publisher_;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr            control_mode_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::MissionResult>::SharedPtr                 mission_result_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr                  home_position_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                      odometry_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPositionSetpoint>::SharedPtr  cmd_pose_subscriber_;

  // callback groups
  // a shared pointer to each callback group has to be saved or the callbacks will never get called
  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  // new callback groups have to be initialized using this function to be saved into callback_groups_
  rclcpp::CallbackGroup::SharedPtr new_cbk_grp();

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

  // subscriber callbacks
  void controlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);
  bool mavsdkLogCallback(const mavsdk::log::Level level, const std::string& message, const std::string& file, const int line);
  void homePositionCallback(const px4_msgs::msg::HomePosition::UniquePtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg);
  void cmdPoseCallback(const px4_msgs::msg::VehicleLocalPositionSetpoint::UniquePtr msg);

  // services provided
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr          arming_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr          takeoff_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr          land_service_;
  rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr             local_waypoint_service_;
  rclcpp::Service<fog_msgs::srv::Path>::SharedPtr             local_path_service_;
  rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr             gps_waypoint_service_;
  rclcpp::Service<fog_msgs::srv::Path>::SharedPtr             gps_path_service_;
  rclcpp::Service<fog_msgs::srv::WaypointToLocal>::SharedPtr  waypoint_to_local_service_;
  rclcpp::Service<fog_msgs::srv::PathToLocal>::SharedPtr      path_to_local_service_;
  rclcpp::Service<fog_msgs::srv::SetPx4ParamInt>::SharedPtr   set_px4_param_int_service_;
  rclcpp::Service<fog_msgs::srv::GetPx4ParamInt>::SharedPtr   get_px4_param_int_service_;
  rclcpp::Service<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_service_;
  rclcpp::Service<fog_msgs::srv::GetPx4ParamFloat>::SharedPtr get_px4_param_float_service_;

  // service clients
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_client_;

  // service callbacks
  bool armingCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool takeoffCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool landCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool localWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request, std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
  bool localPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response);
  bool gpsWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request, std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
  bool gpsPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response);
  bool waypointToLocalCallback(const std::shared_ptr<fog_msgs::srv::WaypointToLocal::Request> request,
                               std::shared_ptr<fog_msgs::srv::WaypointToLocal::Response>      response);
  bool pathToLocalCallback(const std::shared_ptr<fog_msgs::srv::PathToLocal::Request> request, std::shared_ptr<fog_msgs::srv::PathToLocal::Response> response);

  bool setPx4ParamIntCallback(const std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Request> request,
                              std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Response>      response);
  bool setPx4ParamFloatCallback(const std::shared_ptr<fog_msgs::srv::SetPx4ParamFloat::Request> request,
                                std::shared_ptr<fog_msgs::srv::SetPx4ParamFloat::Response>      response);
  bool getPx4ParamIntCallback(const std::shared_ptr<fog_msgs::srv::GetPx4ParamInt::Request> request,
                              std::shared_ptr<fog_msgs::srv::GetPx4ParamInt::Response>      response);
  bool getPx4ParamFloatCallback(const std::shared_ptr<fog_msgs::srv::GetPx4ParamFloat::Request> request,
                                std::shared_ptr<fog_msgs::srv::GetPx4ParamFloat::Response>      response);

  // parameter callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  void publishDiagnostics();

  bool connectPixHawk();
  bool startTakeoff(std::string& fail_reason_out);
  bool startLanding(std::string& fail_reason_out);

  void printAndPublishWaypoints(const std::vector<local_waypoint_t>& wps);

  // timers
  rclcpp::TimerBase::SharedPtr vehicle_state_timer_;
  void vehicleStateRoutine();

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  void diagnosticsRoutine();

  // helper state methods
  void state_mission_finished();
  void state_mission_uploading();
  void state_mission_in_progress();

  void update_vehicle_state(const bool takeoff_started = false, const bool landing_started = false);
  void state_vehicle_not_connected();
  void state_vehicle_not_ready();
  void state_vehicle_takeoff_ready(bool takeoff_started);
  void state_vehicle_taking_off();
  void state_vehicle_autonomous_flight(const bool landing_started);
  void state_vehicle_manual_flight(const bool landing_started);
  void state_vehicle_landing();

  bool gotoAfterTakeoff(std::string& fail_reason_out);

  template<typename T>
  bool startNewMission(const T& path, const uint32_t id, const bool is_global, std::string& fail_reason_out);

  template <typename T>
  mavsdk::Mission::MissionItem to_mission_item(const T& w_in, const bool is_global);
  mavsdk::Mission::MissionItem to_mission_item(const local_waypoint_t& w_in);

  local_waypoint_t to_local_waypoint(const geometry_msgs::msg::PoseStamped& in, const bool is_global);
  local_waypoint_t to_local_waypoint(const mavsdk::Mission::MissionItem& in);
  local_waypoint_t to_local_waypoint(const fog_msgs::srv::WaypointToLocal::Request& in, const bool is_global);
  local_waypoint_t to_local_waypoint(const std::vector<double>& in, const bool is_global);
  local_waypoint_t to_local_waypoint(const Eigen::Vector4d& in, const bool is_global);
};
//}

// --------------------------------------------------------------
// |            ControlInterface class implementation           |
// --------------------------------------------------------------

/* constructor //{ */
ControlInterface::ControlInterface(rclcpp::NodeOptions options) : Node("control_interface", options)
{
  RCLCPP_INFO(get_logger(), "Initializing...");

  try
  {
    uav_name_ = std::string(std::getenv("DRONE_DEVICE_ID"));
  }
  catch (...)
  {
    RCLCPP_ERROR(get_logger(), "Environment variable DRONE_DEVICE_ID was not defined!");
  }
  RCLCPP_INFO(get_logger(), "UAV name is: '%s'", uav_name_.c_str());


  RCLCPP_INFO(get_logger(), "-------------- Loading parameters --------------");

  /* parse params from launch file //{ */
  bool loaded_successfully = true;
  loaded_successfully &= parse_param("device_url", device_url_, *this);
  loaded_successfully &= parse_param("world_frame", world_frame_, *this);
  //}

  /* parse params from config file //{ */
  loaded_successfully &= parse_param("general.octomap_reset_before_takeoff", octomap_reset_before_takeoff_, *this);
  loaded_successfully &= parse_param("general.octomap_reset_timeout", octomap_reset_timeout_, *this);
  loaded_successfully &= parse_param("general.control_update_rate", control_update_rate_, *this);
  loaded_successfully &= parse_param("general.diagnostics_publish_rate", diagnostics_publish_rate_, *this);
  loaded_successfully &= parse_param("general.print_callback_durations", print_callback_durations_, *this);
  loaded_successfully &= parse_param("general.print_callback_min_dur", print_callback_min_dur_, *this);

  loaded_successfully &= parse_param("takeoff.height", takeoff_height_, *this);
  loaded_successfully &= parse_param("takeoff.position_samples", takeoff_position_samples_, *this);

  loaded_successfully &= parse_param("px4.target_velocity", target_velocity_, *this);
  loaded_successfully &= parse_param("px4.waypoint_loiter_time", waypoint_loiter_time_, *this);
  loaded_successfully &= parse_param("px4.waypoint_acceptance_radius", waypoint_acceptance_radius_, *this);
  loaded_successfully &= parse_param("px4.altitude_acceptance_radius", altitude_acceptance_radius_, *this);

  loaded_successfully &= parse_param("mavsdk.logging_print_level", mavsdk_logging_print_level_, *this);
  loaded_successfully &= parse_param("mavsdk.logging_filename", mavsdk_logging_filename_, *this);
  loaded_successfully &= parse_param("mavsdk.heading_offset_correction", heading_offset_correction_, *this);
  loaded_successfully &= parse_param("mavsdk.mission_upload_attempts_threshold", mission_upload_attempts_threshold_, *this);

  if (!loaded_successfully)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  if (control_update_rate_ < 5.0)
  {
    control_update_rate_ = 5.0;
    RCLCPP_WARN(get_logger(), "Control update rate set too slow. Defaulting to 5 Hz");
  }

  //}

  // | ------------- misc. parameters initialization ------------ |
  pose_takeoff_samples_.set_capacity(takeoff_position_samples_);

  /* setup MavSDK logging //{ */
  
  // open and clear the log file first
  if (!mavsdk_logging_filename_.empty())
    mavsdk_logging_file_.open(mavsdk_logging_filename_, std::ios_base::trunc);
  // now bind the callback
  const auto log_cbk = std::bind(&ControlInterface::mavsdkLogCallback, this, _1, _2, _3, _4);
  mavsdk::log::subscribe(log_cbk);
  
  //}

  /* estabilish connection with PX4 //{ */
  mavsdk::ConnectionResult connection_result;
  try
  {
    // Matouš: this shouldn't throw according to the documentation - why is it in a try/catch?
    connection_result = mavsdk_.add_any_connection(device_url_);
  }
  catch (...)
  {
    RCLCPP_ERROR(get_logger(), "Connection failed! Device does not exist: %s", device_url_.c_str());
    exit(EXIT_FAILURE);
  }
  if (connection_result != mavsdk::ConnectionResult::Success)
  {
    RCLCPP_ERROR(get_logger(), "Connection failed: %s", to_string(connection_result).c_str());
    exit(EXIT_FAILURE);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "MAVSDK connected to device: %s", device_url_.c_str());
  }

  //}

  // | ------------------ initialize publishers ----------------- |
  rclcpp::QoS qos(rclcpp::KeepLast(3));
  waypoint_publisher_     = create_publisher<geometry_msgs::msg::PoseArray>("~/waypoints_out", qos);
  diagnostics_publisher_  = create_publisher<fog_msgs::msg::ControlInterfaceDiagnostics>("~/diagnostics_out", qos);
  cmd_pose_publisher_     = create_publisher<geometry_msgs::msg::PoseStamped>("~/cmd_pose_out", qos);

  // service clients
  octomap_reset_client_ = create_client<std_srvs::srv::Empty>("~/octomap_reset_out");

  // | ------------------ initialize callbacks ------------------ |

  parameters_callback_handle_ = add_on_set_parameters_callback(std::bind(&ControlInterface::parametersCallback, this, _1));

  rclcpp::SubscriptionOptions subopts;

  // create a mutually exclusive callback group for each callback so that only a single instance of each callback can be running at one time
  subopts.callback_group = new_cbk_grp();
  control_mode_subscriber_ = create_subscription<px4_msgs::msg::VehicleControlMode>("~/control_mode_in",
      rclcpp::SystemDefaultsQoS(), std::bind(&ControlInterface::controlModeCallback, this, _1), subopts);

  subopts.callback_group = new_cbk_grp();
  home_position_subscriber_ = create_subscription<px4_msgs::msg::HomePosition>("~/home_position_in",
      rclcpp::SystemDefaultsQoS(), std::bind(&ControlInterface::homePositionCallback, this, _1), subopts);

  subopts.callback_group = new_cbk_grp();
  odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>("~/local_odom_in",
      rclcpp::SystemDefaultsQoS(), std::bind(&ControlInterface::odometryCallback, this, _1), subopts);

  subopts.callback_group = new_cbk_grp();
  cmd_pose_subscriber_ = create_subscription<px4_msgs::msg::VehicleLocalPositionSetpoint>("~/cmd_pose_in",
      rclcpp::SystemDefaultsQoS(), std::bind(&ControlInterface::cmdPoseCallback, this, _1), subopts);

  // service handlers
  const auto qos_profile = qos.get_rmw_qos_profile();
  const auto action_grp_ptr = new_cbk_grp();
  arming_service_  = create_service<std_srvs::srv::SetBool>("~/arming_in",
      std::bind(&ControlInterface::armingCallback, this, _1, _2), qos_profile, action_grp_ptr);

  takeoff_service_ = create_service<std_srvs::srv::Trigger>("~/takeoff_in",
      std::bind(&ControlInterface::takeoffCallback, this, _1, _2), qos_profile, action_grp_ptr);

  land_service_ = create_service<std_srvs::srv::Trigger>("~/land_in",
      std::bind(&ControlInterface::landCallback, this, _1, _2), qos_profile, action_grp_ptr);

  const auto waypt_grp_ptr = new_cbk_grp();
  local_waypoint_service_ = create_service<fog_msgs::srv::Vec4>("~/local_waypoint_in",
      std::bind(&ControlInterface::localWaypointCallback, this, _1, _2), qos_profile, waypt_grp_ptr);

  local_path_service_ = create_service<fog_msgs::srv::Path>("~/local_path_in",
      std::bind(&ControlInterface::localPathCallback, this, _1, _2), qos_profile, waypt_grp_ptr);

  gps_waypoint_service_ = create_service<fog_msgs::srv::Vec4>("~/gps_waypoint_in",
      std::bind(&ControlInterface::gpsWaypointCallback, this, _1, _2), qos_profile, waypt_grp_ptr);

  gps_path_service_ = create_service<fog_msgs::srv::Path>("~/gps_path_in",
      std::bind(&ControlInterface::gpsPathCallback, this, _1, _2), qos_profile, waypt_grp_ptr);

  waypoint_to_local_service_ = create_service<fog_msgs::srv::WaypointToLocal>("~/waypoint_to_local_in",
      std::bind(&ControlInterface::waypointToLocalCallback, this, _1, _2), qos_profile, waypt_grp_ptr);

  path_to_local_service_ = create_service<fog_msgs::srv::PathToLocal>("~/path_to_local_in",
      std::bind(&ControlInterface::pathToLocalCallback, this, _1, _2), qos_profile, waypt_grp_ptr);

  const auto param_grp_ptr = new_cbk_grp();
  set_px4_param_int_service_ = create_service<fog_msgs::srv::SetPx4ParamInt>("~/set_px4_param_int_in",
      std::bind(&ControlInterface::setPx4ParamIntCallback, this, _1, _2), qos_profile, param_grp_ptr);
  get_px4_param_int_service_ = create_service<fog_msgs::srv::GetPx4ParamInt>("~/get_px4_param_int_in",
      std::bind(&ControlInterface::getPx4ParamIntCallback, this, _1, _2), qos_profile, param_grp_ptr);
  set_px4_param_float_service_ = create_service<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float_in",
      std::bind(&ControlInterface::setPx4ParamFloatCallback, this, _1, _2), qos_profile, param_grp_ptr);
  get_px4_param_float_service_ = create_service<fog_msgs::srv::GetPx4ParamFloat>("~/get_px4_param_float_in",
      std::bind(&ControlInterface::getPx4ParamFloatCallback, this, _1, _2), qos_profile, param_grp_ptr);

  vehicle_state_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / control_update_rate_),
      std::bind(&ControlInterface::vehicleStateRoutine, this), new_cbk_grp());

  diagnostics_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / diagnostics_publish_rate_),
      std::bind(&ControlInterface::diagnosticsRoutine, this), new_cbk_grp());

  RCLCPP_INFO(get_logger(), "ControlInterface constructor complete.");
}
//}

// --------------------------------------------------------------
// |                 ControlInterfdace callbacks                |
// --------------------------------------------------------------

// | --------------------- Data callbacks --------------------- |

/* controlModeCallback //{ */
void ControlInterface::controlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg)
{
  scope_timer tim(print_callback_durations_, "controlModeCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  if (!system_connected_)
    return;

  getting_control_mode_ = true;

  // evaluate manual control switching (can only be switched to off if vehicle is landed and disarmed)
  if (!manual_override_ && msg->flag_control_manual_enabled)
  {
    RCLCPP_INFO(get_logger(), "Control flag switched to manual. Stopping and clearing mission");
    manual_override_ = true;
    std::string fail_reason = "mission planner not initialized";
    if (!mission_mgr_ || !mission_mgr_->stop_mission(fail_reason))
      RCLCPP_ERROR_STREAM(get_logger(), "Previous mission cannot be stopped (" << fail_reason << "). Manual landing required");
    return;
  }

  if (manual_override_ && !msg->flag_control_manual_enabled)
  {
    const vehicle_state_t state = get_mutexed(state_mutex_, vehicle_state_);
    const bool on_ground_disarmed = !msg->flag_armed && state == vehicle_state_t::not_ready;
    const bool taking_off = state == vehicle_state_t::taking_off;
    if (on_ground_disarmed)
    {
      manual_override_ = false;
      RCLCPP_INFO(get_logger(), "Vehicle is landed and disarmed, enabling automatic control.");
    }
    else if (taking_off)
    {
      manual_override_ = false;
      RCLCPP_INFO(get_logger(), "Vehicle is taking off, enabling automatic control.");
    }
    else
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "NOT enabling automatic control: neither taking off nor landed and disarmed (" << to_string(state) << ")");
  }
}
//}

/* odometryCallback //{ */
void ControlInterface::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg)
{
  scope_timer tim(print_callback_durations_, "odometryCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  const vehicle_state_t state = get_mutexed(state_mutex_, vehicle_state_);
  getting_odom_ = true;
  RCLCPP_INFO_ONCE(get_logger(), "Getting odometry");

  std::scoped_lock lck(pose_mutex_);

  // update the current position and orientation of the vehicle
  pose_pos_.x() = msg->pose.pose.position.x;
  pose_pos_.y() = msg->pose.pose.position.y;
  pose_pos_.z() = msg->pose.pose.position.z;
  pose_ori_.setX(msg->pose.pose.orientation.x);
  pose_ori_.setY(msg->pose.pose.orientation.y);
  pose_ori_.setZ(msg->pose.pose.orientation.z);
  pose_ori_.setW(msg->pose.pose.orientation.w);

  // check if the vehicle is landed and therefore should average its position for takeoff
  switch (state)
  {
    // ignore these states
    vehicle_state_t::invalid:
    vehicle_state_t::taking_off:
    vehicle_state_t::autonomous_flight:
    vehicle_state_t::manual_flight:
    vehicle_state_t::landing:
                      return;
    // add the position sample in these states
    vehicle_state_t::not_connected:
    vehicle_state_t::not_ready:
    vehicle_state_t::takeoff_ready:
                      break;
  }

  // add the current position to the pose samples for takeoff position estimation
  pose_takeoff_samples_.push_back(pose_pos_);
}
//}

  /* cmdPoseCallback //{ */
  void ControlInterface::cmdPoseCallback(const px4_msgs::msg::VehicleLocalPositionSetpoint::UniquePtr msg)
  {
    // convert from NED (north-east-down) coordinates to ENU (east-north-up)
    const Eigen::Vector3d enu_cmd_pos(msg->y, msg->x, -msg->z);
    const double heading_corrected = -msg->yaw - heading_offset_correction_;
    const Eigen::Quaterniond ori(Eigen::AngleAxisd(heading_corrected, Eigen::Vector3d::UnitZ()));

    geometry_msgs::msg::PoseStamped::UniquePtr msg_out = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg_out->header.stamp = get_clock()->now();
    msg_out->header.frame_id = world_frame_;
    msg_out->pose.position.x = enu_cmd_pos.x();
    msg_out->pose.position.y = enu_cmd_pos.y();
    msg_out->pose.position.z = enu_cmd_pos.z();
    msg_out->pose.orientation.x = ori.x();
    msg_out->pose.orientation.y = ori.y();
    msg_out->pose.orientation.z = ori.z();
    msg_out->pose.orientation.w = ori.w();
    cmd_pose_publisher_->publish(std::move(msg_out));

    RCLCPP_INFO_ONCE(get_logger(), "Getting cmd pose, republishing");
  }
  //}

/* homePositionCallback //{ */
void ControlInterface::homePositionCallback(const px4_msgs::msg::HomePosition::UniquePtr msg)
{
  scope_timer tim(print_callback_durations_, "homePositionCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  mavsdk::geometry::CoordinateTransformation::GlobalCoordinate tf;
  tf.latitude_deg  = msg->lat;
  tf.longitude_deg = msg->lon;
  const auto new_tf = std::make_shared<mavsdk::geometry::CoordinateTransformation>(mavsdk::geometry::CoordinateTransformation(tf));
  const Eigen::Vector3d new_home_offset(msg->y, msg->x, -msg->z);

  set_mutexed(coord_transform_mutex_, 
      std::make_tuple(new_tf, new_home_offset),
      std::forward_as_tuple(coord_transform_, home_position_offset_)
    );

  RCLCPP_INFO(get_logger(), "GPS origin set! Lat: %.6f, Lon: %.6f", tf.latitude_deg, tf.longitude_deg);
  RCLCPP_INFO_STREAM(get_logger(), "Home position offset (local): " << new_home_offset.transpose());

  gps_origin_set_ = true;
}
//}

/* mavsdkLogCallback() method //{ */
bool ControlInterface::mavsdkLogCallback(const mavsdk::log::Level level, const std::string& message, const std::string& file, const int line)
{
  scope_timer tim(print_callback_durations_, "mavsdkLogCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(mavsdk_logging_mutex_);
  switch (level)
  {
    case mavsdk::log::Level::Debug:
      if (mavsdk_logging_file_.is_open())
        mavsdk_logging_file_ << "Debug";
      if (mavsdk_logging_print_level_ <= 0)
        RCLCPP_DEBUG_STREAM(get_logger(), "MavSDK: " << message);
      break;
    case mavsdk::log::Level::Info:
      if (mavsdk_logging_file_.is_open())
        mavsdk_logging_file_ << "Info";
      if (mavsdk_logging_print_level_ <= 1)
        RCLCPP_INFO_STREAM(get_logger(), "MavSDK: " << message);
      break;
    case mavsdk::log::Level::Warn:
      if (mavsdk_logging_file_.is_open())
        mavsdk_logging_file_ << "Warning";
      if (mavsdk_logging_print_level_ <= 2)
        RCLCPP_WARN_STREAM(get_logger(), "MavSDK: " << message);
      break;
    case mavsdk::log::Level::Err:
      if (mavsdk_logging_file_.is_open())
        mavsdk_logging_file_ << "Error";
      if (mavsdk_logging_print_level_ <= 3)
        RCLCPP_ERROR_STREAM(get_logger(), "MavSDK: " << message);
      break;
  }
  if (mavsdk_logging_file_.is_open())
    mavsdk_logging_file_ << " (" << file << ":" << line << "): " << message << std::endl;
  // return true so that MavSDK doesn't print out anything
  return true;
}
//}

// | -------------------- Command callbacks ------------------- |

/* takeoffCallback //{ */
bool ControlInterface::takeoffCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response>                       response)
{
  scope_timer tim(print_callback_durations_, "takeoffCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  const auto state = get_mutexed(state_mutex_, vehicle_state_);
  if (state != vehicle_state_t::takeoff_ready)
  {
    response->success = false;
    response->message = "Not taking off, wrong state: " + vehicle_state_str_;
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

  std::string fail_reason;
  if (!startTakeoff(fail_reason))
  {
    response->success = false;
    response->message = "Takeoff rejected: " + fail_reason;
    return true;
  }

  response->success = true;
  response->message = "Taking off";
  std::scoped_lock lck(state_mutex_, telem_mutex_);
  update_vehicle_state(true); // update the vehicle state now as it should change
  return true;
}
//}

/* landCallback //{ */
bool ControlInterface::landCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response>                       response)
{
  scope_timer tim(print_callback_durations_, "landCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  std::string fail_reason;
  if (mission_mgr_ && !mission_mgr_->stop_mission(fail_reason))
  {
    response->success = false;
    response->message = "Cannot land, failed to stop mission: " + fail_reason;
    return true;
  }

  if (!startLanding(fail_reason))
  {
    response->success = false;
    response->message = "Landing rejected: " + fail_reason;
    return true;
  }

  response->success = true;
  response->message = "Landing";
  std::scoped_lock lck(state_mutex_, telem_mutex_);
  update_vehicle_state(false, true);
  return true;
}
//}

/* armingCallback //{ */
bool ControlInterface::armingCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response>                       response)
{
  scope_timer tim(print_callback_durations_, "armingCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  if (!system_connected_)
  {
    response->success = false;
    response->message = "Arming rejected, not initialized";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

  std::scoped_lock lck(action_mutex_);
  if (request->data)
  {
    const auto result = action_->arm();
    if (result != mavsdk::Action::Result::Success)
    {
      response->message = "Arming failed: " + to_string(result);
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }
    else
    {
      response->message = "Vehicle arming";
      response->success = true;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return true;
    }
  }
  else
  {
    const auto result = action_->disarm();
    if (result != mavsdk::Action::Result::Success)
    {
      response->message = "Disarming failed :" + to_string(result);
      response->success = false;
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
      return true;
    }
    else
    {
      response->message = "Vehicle disarming";
      response->success = true;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return true;
    }
  }
}
//}

// | ----------------- Waypoint/path callbacks ---------------- |

/* startNewMission() method //{ */
// helper function to start a new mission immediately
template<typename T>
bool ControlInterface::startNewMission(const T& path, const uint32_t id, const bool is_global, std::string& fail_reason_out)
{
  scope_timer tim(print_callback_durations_, "startNewMission", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  const auto state = get_mutexed(state_mutex_, vehicle_state_);
  // check that we are in a state in which we can add waypoints to the buffer
  if (state != vehicle_state_t::autonomous_flight)
  {
    fail_reason_out = "not flying! Current state is: " + vehicle_state_str_;
    return false;
  }

  // this should never happen if we're in the autonomous_flight state
  if (!mission_mgr_)
  {
    fail_reason_out = "mission planner not initialized!";
    return false;
  }

  // transform the path to a MissionPlan for pixhawk
  mavsdk::Mission::MissionPlan mission_plan;
  mission_plan.mission_items.reserve(path.size());
  for (const auto& pose : path)
    mission_plan.mission_items.push_back(to_mission_item(pose, is_global));

  // ensure that nobody uploads a new mission while after the current one is cancelled and before the new one is started
  std::scoped_lock lck(mission_mgr_->mutex);

  // stop the current mission (if any)
  std::string fail_reason;
  if (!mission_mgr_->stop_mission(fail_reason))
  {
    fail_reason_out = "previous mission cannot be aborted (" + fail_reason + ")";
    return false;
  }

  // finally, let the MissionManager handle the upload & starting of the new mission
  if (!mission_mgr_->new_mission(mission_plan, id, fail_reason))
  {
    fail_reason_out = "new mission could not be added (" + fail_reason + ")";
    return false;
  }

  return true;
}
//}

/* localWaypointCallback //{ */
bool ControlInterface::localWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                             std::shared_ptr<fog_msgs::srv::Vec4::Response>      response)
{
  scope_timer tim(print_callback_durations_, "localWaypointCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);
  if (request->goal.size() != 4)
  {
    response->message = "The waypoint must have 4 coordinates (x, y, z, heading)! Ignoring request.";
    response->success = false;
    RCLCPP_ERROR_STREAM(get_logger(), response->message);
    return true;
  }

  // convert the single waypoint to a path containing a single point
  std::vector<std::vector<double>> path {request->goal};

  // attempt to start the new mission
  std::string reason;
  if (!startNewMission(path, 0, false, reason))
  {
    response->success = false;
    response->message = "Waypoint not set: " + reason;
    RCLCPP_ERROR_STREAM(get_logger(), response->message);
    return true;
  }

  response->success = true;
  response->message = "New waypoint set";
  RCLCPP_INFO_STREAM(get_logger(), response->message);
  return true;
}
//}

/* localPathCallback //{ */
bool ControlInterface::localPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response)
{
  scope_timer tim(print_callback_durations_, "localPathCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  // attempt to start the new mission
  std::string reason;
  if (!startNewMission(request->path.poses, request->mission_id, false, reason))
  {
    response->success = false;
    response->message = "Mission #" + std::to_string(request->mission_id) + ": " + std::to_string(request->path.poses.size()) + " new waypoints not set: " + reason;
    RCLCPP_ERROR_STREAM(get_logger(), response->message);
    return true;
  }

  response->success = true;
  response->message = "Mission #" +  std::to_string(request->mission_id) + ": " + std::to_string(request->path.poses.size()) + " new waypoints set";
  RCLCPP_INFO_STREAM(get_logger(), response->message);
  return true;
}
//}

/* gpsWaypointCallback //{ */
bool ControlInterface::gpsWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                           std::shared_ptr<fog_msgs::srv::Vec4::Response>      response)
{
  scope_timer tim(print_callback_durations_, "gpsWaypointCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);
  if (request->goal.size() != 4)
  {
    response->message = "The waypoint must have 4 coordinates (x, y, z, heading)! Ignoring request.";
    response->success = false;
    RCLCPP_ERROR_STREAM(get_logger(), response->message);
    return true;
  }

  // convert the single waypoint to a path containing a single point
  std::vector<std::vector<double>> path {request->goal};

  // attempt to start the new mission
  std::string reason;
  if (!startNewMission(path, 0, true, reason))
  {
    response->success = false;
    response->message = "Waypoint not set: " + reason;
    RCLCPP_ERROR_STREAM(get_logger(), response->message);
    return true;
  }

  response->success = true;
  response->message = "New waypoint set";
  RCLCPP_INFO_STREAM(get_logger(), response->message);
  return true;
}
//}

/* gpsPathCallback //{ */
bool ControlInterface::gpsPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response)
{
  scope_timer tim(print_callback_durations_, "gpsPathCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  // attempt to start the new mission
  std::string reason;
  if (!startNewMission(request->path.poses, 0, true, reason))
  {
    response->success = false;
    response->message = "Waypoints not set: " + reason;
    RCLCPP_ERROR_STREAM(get_logger(), response->message);
    return true;
  }

  response->success = true;
  response->message = std::to_string(request->path.poses.size()) + " new waypoints set";
  RCLCPP_INFO_STREAM(get_logger(), response->message);
  return true;
}
//}

/* waypointToLocalCallback (gps -> local frame) //{ */
bool ControlInterface::waypointToLocalCallback(const std::shared_ptr<fog_msgs::srv::WaypointToLocal::Request> request,
                                               std::shared_ptr<fog_msgs::srv::WaypointToLocal::Response>      response)
{
  scope_timer tim(print_callback_durations_, "waypointToLocalCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  if (!system_connected_)
  {
    response->success = false;
    response->message = "Cannot transform coordinates, not initialized";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

  if (!gps_origin_set_)
  {
    response->success = false;
    response->message = "Cannot transform coordinates, missing GPS origin";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

  const local_waypoint_t local = to_local_waypoint(*request, true);
  response->local_x = local.x;
  response->local_y = local.y;
  response->local_z = local.z;
  response->heading = local.heading;

  std::stringstream ss;
  ss << "Transformed GPS [" << request->latitude_deg << ", " << request->longitude_deg << ", " << request->relative_altitude_m << "] into local: ["
     << response->local_x << ", " << response->local_y << ", " << response->local_z << "]";
  response->message = ss.str();
  response->success = true;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  return true;
}
//}

/* pathToLocalCallback (gps -> local frame) //{ */
bool ControlInterface::pathToLocalCallback(const std::shared_ptr<fog_msgs::srv::PathToLocal::Request> request,
                                           std::shared_ptr<fog_msgs::srv::PathToLocal::Response>      response)
{
  scope_timer tim(print_callback_durations_, "pathToLocalCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);

  if (!system_connected_)
  {
    response->success = false;
    response->message = "Cannot transform coordinates, not initialized";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

  if (!gps_origin_set_)
  {
    response->success = false;
    response->message = "Cannot transform coordinates, GPS origin not set";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

  nav_msgs::msg::Path local_path;
  for (const auto &pose : request->path.poses)
  {
    const local_waypoint_t wp = to_local_waypoint(pose, true);

    geometry_msgs::msg::Point p_out;
    p_out.x = wp.x;
    p_out.y = wp.y;
    p_out.z = wp.z;

    std::stringstream ss;
    ss << "Transformed GPS [" << pose.pose.position.x << ", " << pose.pose.position.y << "] into local: [" << p_out.x << ", " << p_out.y << "]";
    response->message = ss.str();
    response->success = true;
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());

    geometry_msgs::msg::PoseStamped p_stamped;
    p_stamped.pose.position    = p_out;
    p_stamped.pose.orientation = pose.pose.orientation;
    local_path.poses.push_back(p_stamped);
    local_path.header.frame_id = "local";
    local_path.header.stamp    = get_clock()->now();
  }

  std::stringstream ss;
  ss << "Transformed " << request->path.poses.size() << " GPS poses into " << response->path.poses.size() << " local poses";
  response->path    = local_path;
  response->success = true;
  response->message = ss.str();
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());

  return true;
}
//}

// | --------------- Parameter setting callbacks -------------- |

/* parametersCallback //{ */
rcl_interfaces::msg::SetParametersResult ControlInterface::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
  scope_timer tim(print_callback_durations_, "parametersCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason     = "";

  for (const auto &param : parameters)
  {
    std::stringstream result_ss;

    /* takeoff_height //{ */
    if (param.get_name() == "takeoff.height")
    {
      const double val = param.as_double();
      if (val >= 0.5 && val < 10)
      {
        takeoff_height_   = val;
        result.successful = true;
        RCLCPP_INFO(get_logger(), "Parameter: '%s' set to %1.2f", param.get_name().c_str(), val);
      }
      else
      {
        result_ss << "parameter '" << param.get_name() << "' cannot be set to " << val << " because it is not in range <0.5;10>";
        result.reason = result_ss.str();
      }
    }
    //}

    /* waypoint_loiter_time //{ */
    else if (param.get_name() == "px4.waypoint_loiter_time")
    {
      const double val = param.as_double();
      if (val >= 0.0)
      {
        waypoint_loiter_time_ = val;
        result.successful     = true;
        RCLCPP_INFO(get_logger(), "Parameter: '%s' set to %1.2f", param.get_name().c_str(), val);
      }
      else
      {
        result_ss << "parameter '" << param.get_name() << "' cannot be set to " << val << " because it is a negative value";
        result.reason = result_ss.str();
      }
    }
    //}

    /* reset_octomap_before_takeoff //{ */
    else if (param.get_name() == "general.reset_octomap_before_takeoff")
    {
      octomap_reset_before_takeoff_ = param.as_bool();
      result.successful = true;
      RCLCPP_INFO(get_logger(), "Parameter: '%s' set to %s", param.get_name().c_str(), param.as_bool() ? "TRUE" : "FALSE");
    }
    //}

    else
    {
      result_ss << "parameter '" << param.get_name() << "' cannot be changed dynamically";
      result.reason = result_ss.str();
    }
  }

  if (!result.successful)
    RCLCPP_WARN_STREAM(get_logger(), "Failed to set parameter: " << result.reason);

  return result;
}
//}

/* setPx4ParamIntCallback //{ */
bool ControlInterface::setPx4ParamIntCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Request> request,
                                              std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Response>                       response)
{
  scope_timer tim(print_callback_durations_, "setPx4ParamIntCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(param_mutex_);

  if (!system_connected_)
  {
    response->success = false;
    response->message = "Failed to set PX4 parameter: not initialized";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

  response->param_name = request->param_name;
  response->value      = request->value;

  const auto result = param_->set_param_int(request->param_name, request->value);
  response->success = result == mavsdk::Param::Result::Success;
  if (response->success)
  {
    response->message = "PX4 parameter successfully set";
    RCLCPP_INFO(get_logger(), "PX4 parameter %s successfully set to %ld", request->param_name.c_str(), request->value);
  }
  else
  {
    response->message = "Failed to set PX4 parameter: " + to_string(result);
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }

  return true;
}
//}

/* getPx4ParamIntCallback //{ */
bool ControlInterface::getPx4ParamIntCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::GetPx4ParamInt::Request> request,
                                              std::shared_ptr<fog_msgs::srv::GetPx4ParamInt::Response>                       response)
{
  scope_timer tim(print_callback_durations_, "getPx4ParamIntCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(param_mutex_);

  if (!system_connected_)
  {
    response->success = false;
    response->message = "Failed to read PX4 parameter: not initialized";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

  response->param_name = request->param_name;

  const auto [result, val] = param_->get_param_int(request->param_name);
  response->success = result == mavsdk::Param::Result::Success;
  if (response->success)
  {
    response->message = "PX4 parameter successfully read";
    response->value      = val;
    RCLCPP_INFO(get_logger(), "PX4 parameter %s successfully read with value %ld", request->param_name.c_str(), response->value);
  }
  else
  {
    response->message = "Failed to read PX4 parameter: " + to_string(result);
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }

  return true;
}
//}

/* setPx4ParamFloatCallback //{ */
bool ControlInterface::setPx4ParamFloatCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::SetPx4ParamFloat::Request> request,
                                                std::shared_ptr<fog_msgs::srv::SetPx4ParamFloat::Response>                       response)
{
  scope_timer tim(print_callback_durations_, "setPx4ParamFloatCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(param_mutex_);

  if (!system_connected_)
  {
    response->success = false;
    response->message = "Parameter cannot be set, not initialized";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

  response->param_name = request->param_name;
  response->value      = request->value;

  const auto result = param_->set_param_float(request->param_name, request->value);
  response->success = result == mavsdk::Param::Result::Success;
  if (response->success)
  {
    response->message = "PX4 parameter successfully set";
    RCLCPP_INFO(get_logger(), "PX4 parameter %s successfully set to %f", request->param_name.c_str(), request->value);
  }
  else
  {
    response->message = "Failed to set PX4 parameter: " + to_string(result);
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }

  return true;
}
//}

/* getPx4ParamFloatCallback //{ */
bool ControlInterface::getPx4ParamFloatCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::GetPx4ParamFloat::Request> request,
                                                std::shared_ptr<fog_msgs::srv::GetPx4ParamFloat::Response>                       response)
{
  scope_timer tim(print_callback_durations_, "getPx4ParamFloatCallback", get_logger(), print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(param_mutex_);

  if (!system_connected_)
  {
    response->success = false;
    response->message = "Failed to read PX4 parameter: not initialized";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

  response->param_name = request->param_name;

  const auto [result, val] = param_->get_param_float(request->param_name);
  response->success = result == mavsdk::Param::Result::Success;
  if (response->success)
  {
    response->message = "PX4 parameter successfully read";
    response->value      = val;
    RCLCPP_INFO(get_logger(), "PX4 parameter %s successfully read with value %f", request->param_name.c_str(), response->value);
  }
  else
  {
    response->message = "Failed to read PX4 parameter: " + to_string(result);
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }

  return true;
}
//}

// --------------------------------------------------------------
// |                  CommandInterface routines                 |
// --------------------------------------------------------------

/*   diagnosticsRoutine(); //{ */
void ControlInterface::diagnosticsRoutine()
{
  scope_timer tim(print_callback_durations_, "diagnosticsRoutine", get_logger(), print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(state_mutex_, telem_mutex_);

  // publish some diags
  publishDiagnostics();
}
//}

/* vehicleStateRoutine //{ */
void ControlInterface::vehicleStateRoutine()
{
  scope_timer tim(print_callback_durations_, "vehicleStateRoutine", get_logger(), print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(state_mutex_, telem_mutex_, pose_mutex_);

  const auto prev_state = vehicle_state_;
  update_vehicle_state();
  if (prev_state != vehicle_state_)
    publishDiagnostics();
}
//}

/* update_vehicle_state //{ */

/* the update_vehicle_state() method //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// telem_mutex_
// pose_mutex_
void ControlInterface::update_vehicle_state(const bool takeoff_started, const bool landing_started)
{
  // process the vehicle's state
  switch (vehicle_state_)
  {
    case vehicle_state_t::not_connected:
      state_vehicle_not_connected(); break;
    case vehicle_state_t::not_ready:
      state_vehicle_not_ready(); break;
    case vehicle_state_t::takeoff_ready:
      state_vehicle_takeoff_ready(takeoff_started); break;
    case vehicle_state_t::taking_off:
      state_vehicle_taking_off(); break;
    case vehicle_state_t::autonomous_flight:
      state_vehicle_autonomous_flight(landing_started); break;
    case vehicle_state_t::manual_flight:
      state_vehicle_manual_flight(landing_started); break;
    case vehicle_state_t::landing:
      state_vehicle_landing(); break;
    default:
      assert(false);
      RCLCPP_ERROR(get_logger(), "Invalid vehicle state, this should never happen!");
      return;
  }
}
//}

/* state_vehicle_not_connected() method //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// telem_mutex_
void ControlInterface::state_vehicle_not_connected()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vehicle state: MavSDK system not connected.");
  vehicle_state_str_ = "MavSDK system not connected";

  std::scoped_lock lck(action_mutex_, param_mutex_);
  const bool succ = connectPixHawk();
  // advance to the next state if connection and initialization was OK
  if (succ)
  {
    RCLCPP_INFO(get_logger(), "MavSDK system connected, clearing all missions and switching state to not_ready.");
    std::string reasons = "mission planner not initialized!";
    if (!mission_mgr_->stop_mission(reasons))
      RCLCPP_WARN_STREAM(get_logger(), "Failed to stop mission before transitioning to not_ready (" << reasons << "). Arming may be dangerous!");
    vehicle_state_ = vehicle_state_t::not_ready;
  }
  // otherwise tell the user what we're waiting for
  else
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for MavSDK system connection.");
}
//}

/* state_vehicle_not_ready() method //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// telem_mutex_
// pose_mutex_
void ControlInterface::state_vehicle_not_ready()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vehicle state: not ready for takeoff.");
  vehicle_state_str_ = "not ready for takeoff";

  if (!system_connected_)
  {
    RCLCPP_INFO(get_logger(), "MavSDK system disconnected, switching state to not_connected.");
    vehicle_state_ = vehicle_state_t::not_connected;
    return;
  }

  const bool armed = telem_->armed();
  const auto health = telem_->health();
  const bool healthy = is_healthy(health);
  const auto land_state = telem_->landed_state();

  // advance to the next state if all conditions are met
  if (gps_origin_set_
   && pose_takeoff_samples_.size() >= (size_t)takeoff_position_samples_
   && armed
   && healthy
   && land_state == mavsdk::Telemetry::LandedState::OnGround)
  {
    RCLCPP_INFO(get_logger(), "Vehicle is now ready for takeoff! Switching state to takeoff_ready.");
    vehicle_state_ = vehicle_state_t::takeoff_ready;
  }
  // otherwise tell the user what we're waiting for
  else
  {
    const std::string gps_reason = "insufficient GPS samples (" + std::to_string(pose_takeoff_samples_.size()) + "/" + std::to_string(takeoff_position_samples_) + ")";
    std::string reasons;
    add_reason_if("not armed", !armed, reasons);
    add_unhealthy_reasons(health, reasons);
    add_reason_if("GPS origin not set", !gps_origin_set_, reasons);
    add_reason_if(gps_reason, pose_takeoff_samples_.size() < (size_t)takeoff_position_samples_, reasons);
    add_reason_if("not landed (" + to_string(land_state) + ")", land_state != mavsdk::Telemetry::LandedState::OnGround, reasons);
    vehicle_state_str_ += ", " + reasons;
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Not ready for takeoff: " << reasons);
  }
}
//}

/* state_vehicle_takeoff_ready() method //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// telem_mutex_
// pose_mutex_
void ControlInterface::state_vehicle_takeoff_ready(const bool takeoff_started)
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vehicle state: ready for takeoff.");
  vehicle_state_str_ = "ready for takeoff";

  if (!system_connected_)
  {
    RCLCPP_INFO(get_logger(), "MavSDK system disconnected, switching state to not_connected.");
    vehicle_state_ = vehicle_state_t::not_connected;
    return;
  }

  const bool armed = telem_->armed();
  const auto health = telem_->health();
  const bool healthy = is_healthy(health);
  const auto land_state = telem_->landed_state();

  if (land_state == mavsdk::Telemetry::LandedState::TakingOff || takeoff_started)
  {
    RCLCPP_INFO(get_logger(), "Taking off! Switching state to taking_off.");
    vehicle_state_ = vehicle_state_t::taking_off;
    return;
  }

  if (!armed
   || !healthy
   || land_state != mavsdk::Telemetry::LandedState::OnGround)
  {
    std::string reasons;
    add_reason_if("not armed", !armed, reasons);
    add_unhealthy_reasons(health, reasons);
    add_reason_if("not landed (" + to_string(land_state) + ")", land_state != mavsdk::Telemetry::LandedState::OnGround, reasons);
    RCLCPP_INFO_STREAM(get_logger(), "No longer ready for takeoff: " << reasons << ", stopping all missions and switching state to not_ready.");

    reasons = "mission planner not initialized!";
    if (!mission_mgr_ || !mission_mgr_->stop_mission(reasons))
      RCLCPP_WARN_STREAM(get_logger(), "Failed to stop mission before transitioning to not_ready (" << reasons << "). Arming may be dangerous!");
    pose_takeoff_samples_.clear(); // clear the takeoff pose samples to estimate a new one
    vehicle_state_ = vehicle_state_t::not_ready;
    return;
  }
}
//}

/* state_vehicle_taking_off() method //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// telem_mutex_
// pose_mutex_
void ControlInterface::state_vehicle_taking_off()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vehicle state: taking off.");
  vehicle_state_str_ = "taking off";

  if (!system_connected_)
  {
    RCLCPP_INFO(get_logger(), "Takeoff interrupted: MavSDK system disconnected. Switching state to not_connected.");
    vehicle_state_ = vehicle_state_t::not_connected;
    return;
  }

  const bool armed = telem_->armed();
  const auto land_state = telem_->landed_state();

  if (!armed)
  {
    RCLCPP_INFO(get_logger(), "Takeoff interrupted with disarm. Stopping all missions and switching state to not_ready.");
    std::string reasons = "mission planner not initialized!";
    if (!mission_mgr_ || !mission_mgr_->stop_mission(reasons))
      RCLCPP_WARN_STREAM(get_logger(), "Failed to stop mission before transitioning to not_ready (" << reasons << "). Arming may be dangerous!");
    pose_takeoff_samples_.clear(); // clear the takeoff pose samples to estimate a new one
    vehicle_state_ = vehicle_state_t::not_ready;
    return;
  }

  if (land_state == mavsdk::Telemetry::LandedState::InAir)
  {
    if (manual_override_)
    {
      RCLCPP_INFO(get_logger(), "Takeoff complete. Switching state to manual_flight.");
      vehicle_state_ = vehicle_state_t::manual_flight;
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Takeoff complete. Switching state to autonomous_flight.");
      std::string reasons;
      if (!gotoAfterTakeoff(reasons))
        RCLCPP_WARN_STREAM(get_logger(), "Failed to send the vehicle to the after-takeoff position (" << reasons << ")!");
      vehicle_state_ = vehicle_state_t::autonomous_flight;
    }
    return;
  }
}
//}

/* state_vehicle_autonomous_flight() method //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// telem_mutex_
// pose_mutex_
void ControlInterface::state_vehicle_autonomous_flight(const bool landing_started)
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vehicle state: flying autonomously.");
  vehicle_state_str_ = "autonomous flight";

  if (!system_connected_)
  {
    RCLCPP_INFO(get_logger(), "MavSDK system disconnected, switching state to not_connected.");
    vehicle_state_ = vehicle_state_t::not_connected;
    return;
  }

  const bool armed = telem_->armed();
  const auto land_state = telem_->landed_state();

  if (land_state == mavsdk::Telemetry::LandedState::Landing || landing_started)
  {
    RCLCPP_INFO(get_logger(), "Landing! Switching state to landing.");
    vehicle_state_ = vehicle_state_t::landing;
    return;
  }

  if (!armed
   || land_state != mavsdk::Telemetry::LandedState::InAir)
  {
    std::string reasons;
    add_reason_if("not armed", !armed, reasons);
    add_reason_if("not flying (" + to_string(land_state) + ")", land_state != mavsdk::Telemetry::LandedState::InAir, reasons);
    RCLCPP_INFO_STREAM(get_logger(), "Autonomous flight mode ended: " << reasons << ", stopping all missions and switching state to not_ready.");

    reasons = "mission planner not initialized!";
    if (!mission_mgr_ || !mission_mgr_->stop_mission(reasons))
      RCLCPP_WARN_STREAM(get_logger(), "Failed to stop mission before transitioning to not_ready (" << reasons << "). Arming may be dangerous!");
    pose_takeoff_samples_.clear(); // clear the takeoff pose samples to estimate a new one
    vehicle_state_ = vehicle_state_t::not_ready;
    return;
  }

  if (manual_override_)
  {
    RCLCPP_INFO(get_logger(), "Manual override detected, switching state to manual_flight.");
    vehicle_state_ = vehicle_state_t::manual_flight;
  }
}
//}

/* state_vehicle_manual_flight() method //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// telem_mutex_
// pose_mutex_
void ControlInterface::state_vehicle_manual_flight(const bool landing_started)
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vehicle state: flying manually.");
  vehicle_state_str_ = "manual flight";

  if (!system_connected_)
  {
    RCLCPP_INFO(get_logger(), "MavSDK system disconnected, switching state to not_connected.");
    vehicle_state_ = vehicle_state_t::not_connected;
    return;
  }

  const bool armed = telem_->armed();
  const auto land_state = telem_->landed_state();
  const bool not_flying = land_state != mavsdk::Telemetry::LandedState::InAir && land_state != mavsdk::Telemetry::LandedState::Landing;

  if (landing_started)
  {
    RCLCPP_INFO(get_logger(), "Landing! Switching state to landing.");
    vehicle_state_ = vehicle_state_t::landing;
    return;
  }

  if (!armed
   || not_flying)
  {
    std::string reasons;
    add_reason_if("not armed", !armed, reasons);
    add_reason_if("not flying (" + to_string(land_state) + ")", not_flying, reasons);
    RCLCPP_INFO_STREAM(get_logger(), "Manual flight mode ended: " << reasons << ", stopping all missions and switching state to not_ready.");

    reasons = "mission planner not initialized!";
    if (!mission_mgr_ || !mission_mgr_->stop_mission(reasons))
      RCLCPP_WARN_STREAM(get_logger(), "Failed to stop mission before transitioning to not_ready (" << reasons << "). Arming may be dangerous!");
    pose_takeoff_samples_.clear(); // clear the takeoff pose samples to estimate a new one
    vehicle_state_ = vehicle_state_t::not_ready;
    return;
  }
}
//}

/* state_vehicle_landing() method //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// telem_mutex_
// pose_mutex_
void ControlInterface::state_vehicle_landing()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vehicle state: landing.");
  vehicle_state_str_ = "landing";

  if (!system_connected_)
  {
    RCLCPP_INFO(get_logger(), "MavSDK system disconnected, switching state to not_connected.");
    vehicle_state_ = vehicle_state_t::not_connected;
    return;
  }

  const bool armed = telem_->armed();
  const auto land_state = telem_->landed_state();

  if (!armed)
  {
    RCLCPP_INFO(get_logger(), "Landing interrupted with disarm. Switching state to not_ready.");
    pose_takeoff_samples_.clear(); // clear the takeoff pose samples to estimate a new one
    vehicle_state_ = vehicle_state_t::not_ready;
    return;
  }

  if (land_state == mavsdk::Telemetry::LandedState::OnGround)
  {
    RCLCPP_INFO(get_logger(), "Landing complete. Switching state to not_ready.");
    pose_takeoff_samples_.clear(); // clear the takeoff pose samples to estimate a new one
    vehicle_state_ = vehicle_state_t::not_ready;
    return;
  }

  if (manual_override_)
  {
    RCLCPP_INFO(get_logger(), "Manual override detected, switching state to manual_flight.");
    std::string reasons = "mission planner not initialized!";
    if (!mission_mgr_ || !mission_mgr_->stop_mission(reasons))
      RCLCPP_WARN_STREAM(get_logger(), "Failed to stop mission before transitioning to manual_flight (" << reasons << ")!");
    vehicle_state_ = vehicle_state_t::manual_flight;
  }
}
//}

/* gotoAfterTakeoff method //{ */
// helper method to send the vehicle to the after-takeoff position
// the following mutexes have to be locked by the calling function:
// pose_mutex_
bool ControlInterface::gotoAfterTakeoff(std::string& fail_reason_out)
{
  const auto takeoff_poses = get_mutexed(pose_mutex_, );
  local_waypoint_t goal;

  // average the desired takeoff position
  goal.x = 0;
  goal.y = 0;
  goal.z = 0;
  for (const auto &p : pose_takeoff_samples_)
  {
    goal.x += p.x();
    goal.y += p.y();
    goal.z += p.z();
  }
  goal.x /= pose_takeoff_samples_.size();
  goal.y /= pose_takeoff_samples_.size();
  goal.z /= pose_takeoff_samples_.size();

  goal.z += takeoff_height_;
  goal.heading = quat2heading(pose_ori_);
  if (goal.z > 10.0)
    RCLCPP_WARN(get_logger(), "Takeoff height is too damn high (%.2f)! Height sensor may be malfunctioning.", goal.z);

  const std::vector<local_waypoint_t> path = {goal};
  std::string fail_reason = "mission planner not initialized!";
  return startNewMission(path, 0, false, fail_reason_out);
}
//}

//}

// | ----------- Action and mission related methods ----------- |

/* connectPixHawk //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// action_mutex_
// param_mutex_
// telem_mutex_
bool ControlInterface::connectPixHawk()
{
  if (!system_connected_)
  {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Systems size: %ld", mavsdk_.systems().size());
    if (mavsdk_.systems().empty())
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for connection at URL: %s", device_url_.c_str());
      return false;
    }

    // use the first connected system
    system_ = mavsdk_.systems().front();
    RCLCPP_INFO(get_logger(), "ID: %u", system_->get_system_id());

    // setup the other mavsdk connections
    RCLCPP_INFO(get_logger(), "Target connected");
    action_  = std::make_shared<mavsdk::Action>(system_);
    param_   = std::make_shared<mavsdk::Param>(system_);
    telem_   = std::make_shared<mavsdk::Telemetry>(system_);
    mission_mgr_ = std::make_unique<MissionManager>(mission_upload_attempts_threshold_, system_, get_logger(), get_clock());

    // set the initialized flag to true so that px4 parameters may be initialized
    system_connected_ = true;
  }

  // set default parameters to PX4
  auto request = std::make_shared<fog_msgs::srv::SetPx4ParamFloat::Request>();
  auto response = std::make_shared<fog_msgs::srv::SetPx4ParamFloat::Response>();
  bool succ = true;

  request->param_name = "NAV_ACC_RAD";
  request->value      = waypoint_acceptance_radius_;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Setting %s, value: %f", request->param_name.c_str(), request->value);
  setPx4ParamFloatCallback(request, response);
  succ = succ && response->success;

  request->param_name = "NAV_LOITER_RAD";
  request->value      = waypoint_acceptance_radius_;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Setting %s, value: %f", request->param_name.c_str(), request->value);
  setPx4ParamFloatCallback(request, response);
  succ = succ && response->success;

  request->param_name = "NAV_MC_ALT_RAD";
  request->value      = altitude_acceptance_radius_;
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Setting %s, value: %f", request->param_name.c_str(), request->value);
  setPx4ParamFloatCallback(request, response);
  succ = succ && response->success;

  if (!succ)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "PixHawk initialization failed!");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Initialized");
  return true;
}
//}

/* startTakeoff //{ */
bool ControlInterface::startTakeoff(std::string& fail_reason_out)
{
  std::stringstream ss;
  const auto takeoff_poses = get_mutexed(pose_mutex_, pose_takeoff_samples_);

  std::scoped_lock lck(action_mutex_, telem_mutex_);

  const auto alt_result = action_->set_takeoff_altitude(float(takeoff_height_));
  if (alt_result != mavsdk::Action::Result::Success)
  {
    ss << "failed to set takeoff height " << takeoff_height_ << "m (" << to_string(alt_result) << ")";
    fail_reason_out = ss.str();
    RCLCPP_ERROR_STREAM(get_logger(), "Not taking off: " << fail_reason_out);
    return false;
  }

  if (octomap_reset_before_takeoff_)
  {
    RCLCPP_INFO(get_logger(), "Resetting octomap server");
    const auto reset_req   = std::make_shared<std_srvs::srv::Empty::Request>();
    const auto future_resp = octomap_reset_client_->async_send_request(reset_req);
    const std::chrono::duration<double> timeout_dur(octomap_reset_timeout_);
    if (future_resp.wait_for(timeout_dur) == std::future_status::timeout || future_resp.get() == nullptr)
    {
      fail_reason_out = "failed to reset octomap";
      RCLCPP_ERROR_STREAM(get_logger(), "Not taking off: " << fail_reason_out);
      return false;
    }
  }

  if (takeoff_poses.size() < (size_t)takeoff_position_samples_)
  {
    ss << "need " << takeoff_position_samples_ << " odometry samples, only have " << takeoff_poses.size();
    fail_reason_out = ss.str();
    RCLCPP_WARN_STREAM(get_logger(), "Not taking off: " << fail_reason_out);
    return false;
  }

  if (!telem_->armed())
  {
    ss << "vehicle was disarmed before takeoff action was called";
    fail_reason_out = ss.str();
    RCLCPP_WARN_STREAM(get_logger(), "Not taking off: " << fail_reason_out);
    return false;
  }

  const auto takeoff_result = action_->takeoff();
  if (takeoff_result != mavsdk::Action::Result::Success)
  {
    ss << "takeoff rejected by PixHawk (" << to_string(takeoff_result) << ")";
    fail_reason_out = ss.str();
    RCLCPP_ERROR_STREAM(get_logger(), "Not taking off: " << fail_reason_out);
    return false;
  }

  RCLCPP_WARN(get_logger(), "Takeoff action called.");
  return true;
}
//}

/* startLanding //{ */
bool ControlInterface::startLanding(std::string& fail_reason_out)
{
  std::scoped_lock lck(action_mutex_);
  /* // asynchronous variant prepared for when we implement action server */
  /* action_->land_async([this](const mavsdk::Action::Result res) */
  /*     { */
  /*       if (res != mavsdk::Action::Result::Success) */
  /*         RCLCPP_ERROR_STREAM(get_logger(), "Landing failed: " << to_string(res)); */
  /*       else */
  /*         RCLCPP_INFO(get_logger(), "Landing!"); */
  /*     } */
  /*   ); */
  const auto res = action_->land();
  if (res != mavsdk::Action::Result::Success)
  {
    fail_reason_out = to_string(res);
    RCLCPP_ERROR_STREAM(get_logger(), "Landing failed: " << fail_reason_out.c_str());
    return false;
  }
  RCLCPP_WARN(get_logger(), "Landing action called.");
  return true;
}
//}

// | ---------- Diagnostics and debug helper methods ---------- |

/* publishDiagnostics //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// telem_mutex_
void ControlInterface::publishDiagnostics()
{
  const bool armed = telem_ != nullptr && telem_->armed();

  using msg_t = fog_msgs::msg::ControlInterfaceDiagnostics;
  msg_t msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = world_frame_;

  msg.armed = armed;
  msg.vehicle_state = to_msg(vehicle_state_);
  msg.mission_state = to_msg(mission_mgr_ ? mission_mgr_->state() : mission_state_t::finished);

  msg.mission_id = mission_mgr_ ? mission_mgr_->mission_id() : 0;
  msg.mission_size = mission_mgr_ ? mission_mgr_->mission_size() : 0;
  msg.mission_waypoint = mission_mgr_ ? mission_mgr_->mission_waypoint() : 0;

  msg.gps_origin_set = gps_origin_set_;
  msg.getting_odom = getting_odom_;
  msg.getting_control_mode = getting_control_mode_;

  diagnostics_publisher_->publish(msg);
}
//}

/* printAndPublishWaypoints //{ */
void ControlInterface::printAndPublishWaypoints(const std::vector<local_waypoint_t>& wps)
{
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp    = get_clock()->now();
  msg.header.frame_id = world_frame_;

  for (const auto &w : wps)
  {
    RCLCPP_INFO(get_logger(), "\t[%.2f, %.2f, %.2f, %.2f]", w.x, w.y, w.z, w.heading);
    geometry_msgs::msg::Pose p;
    p.position.x = w.x;
    p.position.y = w.y;
    p.position.z = w.z;
    const Eigen::Quaterniond q(Eigen::AngleAxisd(w.heading, Eigen::Vector3d::UnitZ()));
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    msg.poses.push_back(p);
  }
  waypoint_publisher_->publish(msg);
}
//}

// | --------------------- Utility methods -------------------- |

/* to_mission_item //{ */
template <typename T>
mavsdk::Mission::MissionItem ControlInterface::to_mission_item(const T& w_in, const bool is_global)
{
  const local_waypoint_t w = to_local_waypoint(w_in, is_global);
  return to_mission_item(w);
}

mavsdk::Mission::MissionItem ControlInterface::to_mission_item(const local_waypoint_t& w_in)
{
  local_waypoint_t w = w_in;
  // apply home offset correction
  w.x -= home_position_offset_.x();
  w.y -= home_position_offset_.y();
  w.z -= home_position_offset_.z();

  const auto tf = get_mutexed(coord_transform_mutex_, coord_transform_);
  mavsdk::Mission::MissionItem item;
  gps_waypoint_t global = localToGlobal(tf, w);
  item.latitude_deg = global.latitude;
  item.longitude_deg = global.longitude;
  item.relative_altitude_m = (float)global.altitude;
  item.yaw_deg = float(-radians(global.heading + heading_offset_correction_).convert<degrees>().value());
  item.speed_m_s = float(target_velocity_);  // NAN = use default values. This does NOT limit vehicle max speed

  item.is_fly_through          = true;
  item.gimbal_pitch_deg        = 0.0f;
  item.gimbal_yaw_deg          = 0.0f;
  item.camera_action           = mavsdk::Mission::MissionItem::CameraAction::None;
  item.loiter_time_s           = (float)waypoint_loiter_time_;
  item.camera_photo_interval_s = 0.0f;
  item.acceptance_radius_m     = waypoint_acceptance_radius_;

  return item;
}
//}

/* to_local_waypoint //{ */
local_waypoint_t ControlInterface::to_local_waypoint(const geometry_msgs::msg::PoseStamped& in, const bool is_global)
{
  return to_local_waypoint(Eigen::Vector4d{in.pose.position.x, in.pose.position.y, in.pose.position.z, quat2heading(in.pose.orientation)}, is_global);
}

local_waypoint_t ControlInterface::to_local_waypoint(const mavsdk::Mission::MissionItem& in)
{
  gps_waypoint_t global;
  global.latitude = in.latitude_deg;
  global.longitude = in.longitude_deg;
  global.altitude = in.relative_altitude_m;
  global.heading = degrees(-in.yaw_deg).convert<radians>().value() - heading_offset_correction_;

  const auto tf = get_mutexed(coord_transform_mutex_, coord_transform_);
  local_waypoint_t w = globalToLocal(tf, global);
  // apply home offset correction
  w.x += home_position_offset_.x();
  w.y += home_position_offset_.y();
  w.z += home_position_offset_.z();

  return w;
}

local_waypoint_t ControlInterface::to_local_waypoint(const fog_msgs::srv::WaypointToLocal::Request& in, const bool is_global)
{
  const Eigen::Vector4d as_vec {in.latitude_deg, in.longitude_deg, in.relative_altitude_m, in.heading};
  return to_local_waypoint(as_vec, is_global);
}

local_waypoint_t ControlInterface::to_local_waypoint(const std::vector<double>& in, const bool is_global)
{
  assert(in.size() == 4);
  const Eigen::Vector4d as_vec {in.at(0), in.at(1), in.at(2), in.at(3)};
  return to_local_waypoint(as_vec, is_global);
}

local_waypoint_t ControlInterface::to_local_waypoint(const Eigen::Vector4d& in, const bool is_global)
{
  if (is_global)
  {
    const auto tf = get_mutexed(coord_transform_mutex_, coord_transform_);
    gps_waypoint_t wp;
    wp.latitude  = in.x();
    wp.longitude = in.y();
    wp.altitude  = in.z();
    wp.heading       = in.w();
    return globalToLocal(tf, wp);
  }
  else
  {
    local_waypoint_t wp;
    wp.x   = in.x();
    wp.y   = in.y();
    wp.z   = in.z();
    wp.heading = in.w();
    return wp;
  }
}
//}

/* new_cbk_grp() method //{ */
// just a util function that returns a new mutually exclusive callback group to shorten the call
rclcpp::CallbackGroup::SharedPtr ControlInterface::new_cbk_grp()
{
  const rclcpp::CallbackGroup::SharedPtr new_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_groups_.push_back(new_group);
  return new_group;
}
//}

//}

}  // namespace control_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(control_interface::ControlInterface)
