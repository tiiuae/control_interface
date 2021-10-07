#include <deque>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <fog_msgs/msg/control_interface_diagnostics.hpp>
#include <fog_msgs/srv/get_bool.hpp>
#include <fog_msgs/srv/get_origin.hpp>
#include <fog_msgs/srv/get_px4_param_int.hpp>
#include <fog_msgs/srv/path.hpp>
#include <fog_msgs/srv/path_to_local.hpp>
#include <fog_msgs/srv/set_px4_param_float.hpp>
#include <fog_msgs/srv/set_px4_param_int.hpp>
#include <fog_msgs/srv/vec4.hpp>
#include <fog_msgs/srv/waypoint_to_local.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mavsdk/geometry.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/param/param.h>
/* #include <mavsdk/plugins/telemetry/telemetry.h> */
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/mission_result.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::placeholders;

namespace control_interface
{

struct local_waypoint_t
{
  double x;
  double y;
  double z;
  double yaw;
};

struct gps_waypoint_t
{
  double latitude;
  double longitude;
  double altitude;
  double yaw;
};

/* getYaw //{ */
double getYaw(const tf2::Quaternion &q) {
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

double getYaw(const geometry_msgs::msg::Quaternion &q) {
  tf2::Quaternion tq;
  tq.setX(q.x);
  tq.setY(q.y);
  tq.setZ(q.z);
  tq.setW(q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}
//}

/* angle conversions //{ */
double radToDeg(const double &angle_rad) {
  return angle_rad * 180.0 / M_PI;
}

double degToRad(const double &angle_deg) {
  return angle_deg * M_PI / 180.0;
}
//}

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
  wl.yaw = wg.yaw;
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
  wg.yaw        = wl.yaw;
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

/* class ControlInterface //{ */
class ControlInterface : public rclcpp::Node {
public:
  ControlInterface(rclcpp::NodeOptions options);

private:
  std::atomic_bool is_initialized_       = false;
  std::atomic_bool getting_landed_info_  = false;
  std::atomic_bool getting_control_mode_ = false;
  std::atomic_bool start_mission_        = false;
  std::atomic_bool armed_                = false;
  std::atomic_bool takeoff_called_       = false;
  std::atomic_bool takeoff_completed_    = false;
  std::atomic_bool motion_started_       = false;
  std::atomic_bool landed_               = true;

  std::atomic_bool manual_control_flag_ = true;
  std::atomic_bool stop_commanding_     = false;

  std::atomic_bool gps_origin_set_      = false;
  std::atomic_bool gps_origin_called_   = false;
  std::atomic_bool getting_odom_        = false;
  std::atomic_bool getting_odom_called_ = false;

  std::atomic_bool mission_finished_      = true;
  unsigned         last_mission_instance_ = 1;

  std::string uav_name_    = "";
  std::string world_frame_ = "";

  std::string                      device_url_;
  mavsdk::Mavsdk                   mavsdk_;
  std::shared_ptr<mavsdk::System>  system_;
  std::shared_ptr<mavsdk::Action>  action_;
  std::shared_ptr<mavsdk::Mission> mission_;
  std::shared_ptr<mavsdk::Param>   param_;
  mavsdk::Mission::MissionPlan     mission_plan_;
  std::mutex                       mission_mutex_;
  /* std::shared_ptr<mavsdk::Telemetry> telemetry_; */

  std::mutex                   waypoint_buffer_mutex_;
  std::deque<local_waypoint_t> waypoint_buffer_;
  Eigen::Vector4d              desired_pose_;
  Eigen::Vector2d              home_position_offset_ = Eigen::Vector2d(0, 0);

  // use takeoff lat and long to initialize local frame
  std::shared_ptr<mavsdk::geometry::CoordinateTransformation> coord_transform_;
  std::mutex                                                  coord_transform_mutex_;

  // config params
  double yaw_offset_correction_        = M_PI / 2;
  double takeoff_height_               = 2.5;
  double waypoint_marker_scale_        = 0.3;
  double control_update_rate_          = 10.0;
  double waypoint_loiter_time_         = 0.0;
  bool   reset_octomap_before_takeoff_ = true;
  float  waypoint_acceptance_radius_   = 0.3f;
  double target_velocity_              = 1.0;
  size_t takeoff_position_samples_     = 20;

  // vehicle local position
  std::vector<Eigen::Vector3d> pos_samples_;
  float                        pos_[3];
  tf2::Quaternion              ori_;

  // publishers
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr   vehicle_command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_publisher_;  // https://ctu-mrs.github.io/docs/system/relative_commands.html
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   waypoint_marker_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   waypoint_there_and_back_publisher_;

  rclcpp::Publisher<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr diagnostics_publisher_;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr  control_mode_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::MissionResult>::SharedPtr       mission_result_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr        home_position_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odometry_subscriber_;

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // subscriber callbacks
  void controlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);
  void landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg);
  void missionResultCallback(const px4_msgs::msg::MissionResult::UniquePtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg);
  /* void homeCallback(const mavsdk::Telemetry::Position home_position); */
  void homePositionCallback(const px4_msgs::msg::HomePosition::UniquePtr msg);

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
  rclcpp::Service<fog_msgs::srv::SetPx4ParamInt>::SharedPtr   set_px4_param_int_;
  rclcpp::Service<fog_msgs::srv::GetPx4ParamInt>::SharedPtr   get_px4_param_int_;
  rclcpp::Service<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_;

  // service clients
  rclcpp::Client<fog_msgs::srv::GetOrigin>::SharedPtr        get_origin_client_;
  rclcpp::Client<fog_msgs::srv::GetBool>::SharedPtr          getting_odom_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr            octomap_reset_client_;
  rclcpp::Client<fog_msgs::srv::SetPx4ParamFloat>::SharedPtr set_px4_param_float_client_;

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
  bool gpsOriginCallback(rclcpp::Client<fog_msgs::srv::GetOrigin>::SharedFuture future);
  bool odomAvailableCallback(rclcpp::Client<fog_msgs::srv::GetBool>::SharedFuture future);

  // parameter callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  void printSensorsStatus();
  void publishDiagnostics();

  bool takeoff();
  bool land();
  bool startMission();
  bool uploadMission();
  bool stopPreviousMission();

  void addToMission(local_waypoint_t w);
  void publishDebugMarkers();
  void publishDesiredPose();

  std_msgs::msg::ColorRGBA generateColor(const double r, const double g, const double b, const double a);

  // timers
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr     control_timer_;
  void                             controlRoutine(void);

  // utils
  template <class T>
  bool parse_param(const std::string &param_name, T &param_dest);
};
//}

/* constructor //{ */
ControlInterface::ControlInterface(rclcpp::NodeOptions options) : Node("control_interface", options) {

  RCLCPP_INFO(this->get_logger(), "[%s]: Initializing...", this->get_name());

  try {
    uav_name_ = std::string(std::getenv("DRONE_DEVICE_ID"));
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Environment variable DRONE_DEVICE_ID was not defined!", this->get_name());
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: UAV name is: '%s'", this->get_name(), uav_name_.c_str());

  /* parse params from config file //{ */
  parse_param("device_url", device_url_);
  parse_param("yaw_offset_correction", yaw_offset_correction_);
  parse_param("takeoff_height", takeoff_height_);
  parse_param("waypoint_marker_scale", waypoint_marker_scale_);
  parse_param("waypoint_loiter_time", waypoint_loiter_time_);
  parse_param("reset_octomap_before_takeoff", reset_octomap_before_takeoff_);
  parse_param("waypoint_acceptance_radius", waypoint_acceptance_radius_);
  parse_param("target_velocity", target_velocity_);
  parse_param("control_update_rate", control_update_rate_);
  parse_param("takeoff_position_samples", takeoff_position_samples_);

  if (control_update_rate_ < 5.0) {
    control_update_rate_ = 5.0;
    RCLCPP_WARN(this->get_logger(), "[%s]: Control update rate set too slow. Defaulting to 5 Hz", this->get_name());
  }

  world_frame_ = "world";  // TODO FIXME hardcoded??

  //}

  /* estabilish connection with PX4 //{ */
  mavsdk::ConnectionResult connection_result;
  try {
    connection_result = mavsdk_.add_any_connection(device_url_);
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Connection failed! Device does not exist: %s", this->get_name(), device_url_.c_str());
    exit(EXIT_FAILURE);
  }
  if (connection_result != mavsdk::ConnectionResult::Success) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Connection failed: %s", this->get_name(), connection_result);
    exit(EXIT_FAILURE);
  } else {
    RCLCPP_INFO(this->get_logger(), "[%s]: MAVSDK connected to device: %s", this->get_name(), device_url_.c_str());
  }

  bool connected = false;
  while (rclcpp::ok() && !connected) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Systems size: %ld", this->get_name(), mavsdk_.systems().size());
    if (mavsdk_.systems().size() < 1) {
      RCLCPP_INFO(this->get_logger(), "[%s]: Waiting for connection at URL: %s", this->get_name(), device_url_.c_str());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
    for (unsigned i = 0; i < mavsdk_.systems().size(); i++) {
      if (mavsdk_.systems().at(i)->get_system_id() == 1) {
        RCLCPP_INFO(this->get_logger(), "[%s]: ID: %u", this->get_name(), mavsdk_.systems().at(i)->get_system_id());
        connected = true;
        system_   = mavsdk_.systems().at(i);
        break;
      }
    }
  }

  if (!rclcpp::ok())
    return;

  RCLCPP_INFO(this->get_logger(), "[%s]: Target connected", this->get_name());
  action_  = std::make_shared<mavsdk::Action>(system_);
  mission_ = std::make_shared<mavsdk::Mission>(system_);
  param_   = std::make_shared<mavsdk::Param>(system_);
  /* telemetry_ = std::make_shared<mavsdk::Telemetry>(system_); */
  /* telemetry_->subscribe_home(std::bind(&ControlInterface::homeCallback, this, _1)); */
  //}

  callback_group_        = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_opt           = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;

  rclcpp::QoS qos(rclcpp::KeepLast(3));
  // publishers
  vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("~/vehicle_command_out", qos);
  desired_pose_publisher_    = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/desired_pose_out", qos);
  waypoint_marker_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("~/waypoint_markers_out", qos);
  diagnostics_publisher_     = this->create_publisher<fog_msgs::msg::ControlInterfaceDiagnostics>("~/diagnostics_out", qos);

  // subscribers
  control_mode_subscriber_  = this->create_subscription<px4_msgs::msg::VehicleControlMode>("~/control_mode_in", rclcpp::SystemDefaultsQoS(),
                                                                                          std::bind(&ControlInterface::controlModeCallback, this, _1), sub_opt);
  land_detected_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "~/land_detected_in", rclcpp::SystemDefaultsQoS(), std::bind(&ControlInterface::landDetectedCallback, this, _1), sub_opt);
  mission_result_subscriber_ = this->create_subscription<px4_msgs::msg::MissionResult>("~/mission_result_in", rclcpp::SystemDefaultsQoS(),
                                                                                       std::bind(&ControlInterface::missionResultCallback, this, _1), sub_opt);
  home_position_subscriber_  = this->create_subscription<px4_msgs::msg::HomePosition>("~/home_position_in", rclcpp::SystemDefaultsQoS(),
                                                                                     std::bind(&ControlInterface::homePositionCallback, this, _1), sub_opt);
  odometry_subscriber_       = this->create_subscription<nav_msgs::msg::Odometry>("~/local_odom_in", rclcpp::SystemDefaultsQoS(),
                                                                            std::bind(&ControlInterface::odometryCallback, this, _1), sub_opt);

  callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ControlInterface::parametersCallback, this, _1));

  // service clients
  get_origin_client_    = this->create_client<fog_msgs::srv::GetOrigin>("~/get_origin");
  getting_odom_client_  = this->create_client<fog_msgs::srv::GetBool>("~/getting_odom");
  octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>("~/octomap_reset_out");

  // service handlers
  arming_service_  = this->create_service<std_srvs::srv::SetBool>("~/arming_in", std::bind(&ControlInterface::armingCallback, this, _1, _2),
                                                                 qos.get_rmw_qos_profile(), callback_group_);
  takeoff_service_ = this->create_service<std_srvs::srv::Trigger>("~/takeoff_in", std::bind(&ControlInterface::takeoffCallback, this, _1, _2),
                                                                  qos.get_rmw_qos_profile(), callback_group_);
  land_service_ = this->create_service<std_srvs::srv::Trigger>("~/land_in", std::bind(&ControlInterface::landCallback, this, _1, _2), qos.get_rmw_qos_profile(),
                                                               callback_group_);
  local_waypoint_service_ = this->create_service<fog_msgs::srv::Vec4>("~/local_waypoint_in", std::bind(&ControlInterface::localWaypointCallback, this, _1, _2),
                                                                      qos.get_rmw_qos_profile(), callback_group_);
  local_path_service_     = this->create_service<fog_msgs::srv::Path>("~/local_path_in", std::bind(&ControlInterface::localPathCallback, this, _1, _2),
                                                                  qos.get_rmw_qos_profile(), callback_group_);
  gps_waypoint_service_   = this->create_service<fog_msgs::srv::Vec4>("~/gps_waypoint_in", std::bind(&ControlInterface::gpsWaypointCallback, this, _1, _2),
                                                                    qos.get_rmw_qos_profile(), callback_group_);
  gps_path_service_       = this->create_service<fog_msgs::srv::Path>("~/gps_path_in", std::bind(&ControlInterface::gpsPathCallback, this, _1, _2),
                                                                qos.get_rmw_qos_profile(), callback_group_);
  waypoint_to_local_service_ = this->create_service<fog_msgs::srv::WaypointToLocal>(
      "~/waypoint_to_local_in", std::bind(&ControlInterface::waypointToLocalCallback, this, _1, _2), qos.get_rmw_qos_profile(), callback_group_);
  path_to_local_service_ = this->create_service<fog_msgs::srv::PathToLocal>(
      "~/path_to_local_in", std::bind(&ControlInterface::pathToLocalCallback, this, _1, _2), qos.get_rmw_qos_profile(), callback_group_);
  set_px4_param_int_ =
      this->create_service<fog_msgs::srv::SetPx4ParamInt>("~/set_px4_param_int", std::bind(&ControlInterface::setPx4ParamIntCallback, this, _1, _2));
  get_px4_param_int_ =
      this->create_service<fog_msgs::srv::GetPx4ParamInt>("~/get_px4_param_int", std::bind(&ControlInterface::getPx4ParamIntCallback, this, _1, _2));
  set_px4_param_float_ =
      this->create_service<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float", std::bind(&ControlInterface::setPx4ParamFloatCallback, this, _1, _2));

  set_px4_param_float_client_ = this->create_client<fog_msgs::srv::SetPx4ParamFloat>("~/set_px4_param_float");

  control_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / control_update_rate_), std::bind(&ControlInterface::controlRoutine, this), callback_group_);

  octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>("~/octomap_reset_out");

  desired_pose_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);

  auto request        = std::make_shared<fog_msgs::srv::SetPx4ParamFloat::Request>();
  request->param_name = "NAV_ACC_RAD";
  request->value      = waypoint_acceptance_radius_;
  RCLCPP_INFO(this->get_logger(), "[%s]: Setting %s, value: %f", this->get_name(), request->param_name.c_str(), request->value);
  auto call_result = set_px4_param_float_client_->async_send_request(request);

  is_initialized_.store(true);
  RCLCPP_INFO(this->get_logger(), "[%s]: Initialized", this->get_name());
}
//}

/* parametersCallback //{ */
rcl_interfaces::msg::SetParametersResult ControlInterface::parametersCallback(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason     = "";
  char buff[300];

  for (const auto &param : parameters) {

    /* takeoff_height //{ */
    if (param.get_name() == "takeoff_height") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.5 && param.as_double() < 10) {
          takeoff_height_   = param.as_double();
          result.successful = true;
          RCLCPP_INFO(this->get_logger(), "[%s]: Parameter: '%s' set to %1.2f", this->get_name(), param.get_name().c_str(), param.as_double());
        } else {
          snprintf(buff, sizeof(buff),
                   "parameter '%s' cannot be set to %1.2f because it is not in "
                   "range <0.5;10>",
                   param.get_name().c_str(), param.as_double());
          result.reason = buff;
        }
      } else {
        snprintf(buff, sizeof(buff), "parameter '%s' has to be type DOUBLE", param.get_name().c_str());
        result.reason = buff;
      }
      //}

      /* waypoint_marker_scale //{ */
    } else if (param.get_name() == "waypoint_marker_scale") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() > 0.0) {
          waypoint_marker_scale_ = param.as_double();
          result.successful      = true;
          RCLCPP_INFO(this->get_logger(), "[%s]: Parameter: '%s' set to %1.2f", this->get_name(), param.get_name().c_str(), param.as_double());
        } else {
          snprintf(buff, sizeof(buff), "parameter '%s' cannot be set to %1.2f because it is not >0", param.get_name().c_str(), param.as_double());
          result.reason = buff;
        }
      } else {
        snprintf(buff, sizeof(buff), "parameter '%s' has to be type DOUBLE", param.get_name().c_str());
        result.reason = buff;
      }
      //}

      /* waypoint_loiter_time //{ */
    } else if (param.get_name() == "waypoint_loiter_time") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0) {
          waypoint_loiter_time_ = param.as_double();
          result.successful     = true;
          RCLCPP_INFO(this->get_logger(), "[%s]: Parameter: '%s' set to %1.2f", this->get_name(), param.get_name().c_str(), param.as_double());
        } else {
          snprintf(buff, sizeof(buff),
                   "parameter '%s' cannot be set to %1.2f because it is a "
                   "negative value",
                   param.get_name().c_str(), param.as_double());
          result.reason = buff;
        }
      } else {
        snprintf(buff, sizeof(buff), "parameter '%s' has to be type DOUBLE", param.get_name().c_str());
        result.reason = buff;
      }
      //}

      /* reset_octomap_before_takeoff //{ */
    } else if (param.get_name() == "reset_octomap_before_takeoff") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        reset_octomap_before_takeoff_ = param.as_bool();
        result.successful             = true;
        RCLCPP_INFO(this->get_logger(), "[%s]: Parameter: '%s' set to %s", this->get_name(), param.get_name().c_str(), param.as_bool() ? "TRUE" : "FALSE");
      } else {
        snprintf(buff, sizeof(buff), "parameter '%s' has to be type BOOL", param.get_name().c_str());
        result.reason = buff;
      }
      //}

    } else {
      snprintf(buff, sizeof(buff), "parameter '%s' cannot be changed dynamically", param.get_name().c_str());
      result.reason = buff;
    }
  }

  if (!result.successful) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Failed to set parameter: %s", this->get_name(), result.reason.c_str());
  }

  return result;
}
//}

/* controlModeCallback //{ */
void ControlInterface::controlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg) {
  if (!is_initialized_.load()) {
    return;
  }

  getting_control_mode_.store(true);

  bool previous_manual_flag = manual_control_flag_.load();
  bool current_manual_flag  = msg->flag_control_manual_enabled;

  if (!previous_manual_flag && current_manual_flag) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Control flag switched to manual. Stop commanding", this->get_name());
    stop_commanding_.store(true);
    if (!stopPreviousMission()) {
      RCLCPP_ERROR(this->get_logger(), "[%s]: Previous mission cannot be stopped. Manual landing required", this->get_name());
    }
  }

  manual_control_flag_.store(msg->flag_control_manual_enabled);

  if (armed_.load() != msg->flag_armed) {
    armed_.store(msg->flag_armed);
    if (armed_.load()) {
      RCLCPP_WARN(this->get_logger(), "[%s]: Vehicle armed", this->get_name());
    } else {
      start_mission_.store(false);
      motion_started_.store(false);
      RCLCPP_WARN(this->get_logger(), "[%s]: Vehicle disarmed", this->get_name());
      if (landed_.load() && stop_commanding_.load()) {
        RCLCPP_INFO(this->get_logger(), "[%s]: Auto control will be enabled", this->get_name());
        stop_commanding_.store(false);
      }
    }
  }
}
//}

/* landDetectedCallback //{ */
void ControlInterface::landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg) {
  if (!is_initialized_.load()) {
    return;
  }
  getting_landed_info_.store(true);
  // checking only ground_contact flag instead of landed due to a problem in
  // simulation
  landed_.store(msg->ground_contact);
}
//}

/* missionResultCallback //{ */
void ControlInterface::missionResultCallback(const px4_msgs::msg::MissionResult::UniquePtr msg) {
  if (!is_initialized_.load()) {
    return;
  }

  unsigned instance_count = msg->instance_count;

  if (msg->finished && instance_count != last_mission_instance_) {
    mission_finished_.store(true);
    last_mission_instance_ = msg->instance_count;
  }
}
//}

/* homePositionCallback //{ */
void ControlInterface::homePositionCallback(const px4_msgs::msg::HomePosition::UniquePtr msg) {
  if (!is_initialized_.load()) {
    return;
  }

  /* if (!gps_origin_set_.load()) { */
  mavsdk::geometry::CoordinateTransformation::GlobalCoordinate ref;
  ref.latitude_deg  = msg->lat;
  ref.longitude_deg = msg->lon;

  {
    std::scoped_lock lock(coord_transform_mutex_);
    coord_transform_ = std::make_shared<mavsdk::geometry::CoordinateTransformation>(mavsdk::geometry::CoordinateTransformation(ref));
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: GPS origin set! Lat: %.6f, Lon: %.6f", this->get_name(), ref.latitude_deg, ref.longitude_deg);

  home_position_offset_ = Eigen::Vector2d(msg->x, msg->y);
  RCLCPP_INFO(this->get_logger(), "[%s]: Home position offset (local): %.2f, %.2f", this->get_name(), home_position_offset_.y(), home_position_offset_.x());

  gps_origin_set_.store(true);
  /* } */
}
//}

/* odometryCallback //{ */
void ControlInterface::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg) {
  if (!is_initialized_.load()) {
    return;
  }

  getting_odom_.store(true);
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting odometry", this->get_name());

  pos_[0] = msg->pose.pose.position.x;
  pos_[1] = msg->pose.pose.position.y;
  pos_[2] = msg->pose.pose.position.z;
  ori_.setX(msg->pose.pose.orientation.x);
  ori_.setY(msg->pose.pose.orientation.y);
  ori_.setZ(msg->pose.pose.orientation.z);
  ori_.setW(msg->pose.pose.orientation.w);

  Eigen::Vector3d pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  pos_samples_.push_back(pos);
  if (pos_samples_.size() > takeoff_position_samples_) {
    pos_samples_.erase(pos_samples_.begin());
  }

  if (takeoff_called_.load() && !stop_commanding_.load()) {
    if (std::abs(msg->pose.pose.position.z - desired_pose_.z()) < 0.1) {
      RCLCPP_INFO(this->get_logger(), "[ControlInterface]: Takeoff completed");
      takeoff_completed_.store(true);
      takeoff_called_.store(false);
    }
  }

  /* desired_pose_.x() = pos_[0]; */
  /* desired_pose_.y() = pos_[1]; */
  /* desired_pose_.z() = pos_[2]; */
  /* desired_pose_.w() = getYaw(ori_); */
}
//}

/* takeoffCallback //{ */
bool ControlInterface::takeoffCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response>                       response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Takeoff rejected, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gps_origin_set_.load()) {
    response->success = false;
    response->message = "Takeoff rejected, GPS origin not set";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!armed_.load()) {
    response->success = false;
    response->message = "Takeoff rejected, vehicle not armed";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!landed_.load()) {
    response->success = false;
    response->message = "Takeoff rejected, vehicle not landed";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  bool success = takeoff();
  if (success) {
    response->success = true;
    response->message = "Taking off";
    return true;
  }

  response->success = false;
  response->message = "Takeoff rejected";
  return true;
}
//}

/* landCallback //{ */
bool ControlInterface::landCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response>                       response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Landing rejected, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gps_origin_set_.load()) {
    response->success = false;
    response->message = "Landing rejected, GPS origin not set";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!armed_.load()) {
    response->success = false;
    response->message = "Landing rejected, vehicle not armed";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (landed_.load()) {
    response->success = false;
    response->message = "Landing rejected, vehicle not airborne";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  bool success = stopPreviousMission() && land();
  if (success) {
    response->success = true;
    response->message = "Landing";
    return true;
  }
  response->success = false;
  response->message = "Landing rejected";
  return true;
}
//}

/* armingCallback //{ */
bool ControlInterface::armingCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response>                       response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Arming rejected, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  /* if (!gps_origin_set_.load()) { */
  /*   response->success = false; */
  /*   response->message = "Arming rejected, GPS origin not set"; */
  /*   RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str()); */
  /*   return true; */
  /* } */

  if (request->data) {
    auto result = action_->arm();
    if (result != mavsdk::Action::Result::Success) {
      response->message = "Arming failed";
      response->success = false;
      RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
      return true;
    } else {
      response->message = "Vehicle armed";
      response->success = true;
      armed_.store(true);
      RCLCPP_WARN(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
      return true;
    }
  } else {
    auto result = action_->disarm();
    if (result != mavsdk::Action::Result::Success) {
      response->message = "Disarming failed";
      response->success = false;
      RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
      return true;
    } else {
      response->message = "Vehicle disarmed";
      response->success = true;
      armed_.store(false);
      takeoff_completed_.store(false);
      RCLCPP_WARN(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
      return true;
    }
  }
}
//}

/* localWaypointCallback //{ */
bool ControlInterface::localWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                             std::shared_ptr<fog_msgs::srv::Vec4::Response>      response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Waypoint not set, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gps_origin_set_.load()) {
    response->success = false;
    response->message = "Waypoint not set, missing GPS origin";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (landed_.load()) {
    response->success = false;
    response->message = "Waypoint not set, vehicle not airborne";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (stop_commanding_.load()) {
    response->success = false;
    response->message = "Waypoint not set, vehicle is under manual control";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!takeoff_completed_.load()) {
    response->success = false;
    response->message = "Waypoint not set, vehicle not flying normally";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!stopPreviousMission()) {
    response->success = false;
    response->message = "Waypoint not set, previous mission cannot be aborted";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  response->message = "Waypoint set";
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());

  local_waypoint_t w;
  w.x   = request->goal[0];
  w.y   = request->goal[1];
  w.z   = request->goal[2];
  w.yaw = request->goal[3];
  {
    std::scoped_lock lock(waypoint_buffer_mutex_);
    waypoint_buffer_.push_back(w);
  }
  motion_started_.store(true);
  return true;
}
//}

/* localPathCallback //{ */
bool ControlInterface::localPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Waypoints not set, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gps_origin_set_.load()) {
    response->success = false;
    response->message = "Waypoints not set, missing GPS origin";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (stop_commanding_.load()) {
    response->success = false;
    response->message = "Waypoints not set, vehicle is under manual control";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!takeoff_completed_.load()) {
    response->success = false;
    response->message = "Waypoints not set, vehicle not flying normally";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (request->path.poses.size() < 1) {
    response->success = false;
    response->message = "Waypoints not set, request is empty";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!stopPreviousMission()) {
    response->success = false;
    response->message = "Waypoints not set, previous mission cannot be aborted";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  {
    std::scoped_lock lock(waypoint_buffer_mutex_);

    RCLCPP_INFO(this->get_logger(), "[%s]: Got %d waypoints", this->get_name(), request->path.poses.size());
    for (size_t i = 0; i < request->path.poses.size(); i++) {
      local_waypoint_t w;
      w.x   = request->path.poses[i].pose.position.x;
      w.y   = request->path.poses[i].pose.position.y;
      w.z   = request->path.poses[i].pose.position.z;
      w.yaw = getYaw(request->path.poses[i].pose.orientation);
      waypoint_buffer_.push_back(w);
    }
  }
  motion_started_.store(true);
  response->success = true;
  response->message = "Waypoints set";
  return true;
}
//}

/* gpsWaypointCallback //{ */
bool ControlInterface::gpsWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                           std::shared_ptr<fog_msgs::srv::Vec4::Response>      response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Waypoint not set, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gps_origin_set_.load()) {
    response->success = false;
    response->message = "Waypoint not set, missing GPS origin";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (landed_.load()) {
    response->success = false;
    response->message = "Waypoint not set, vehicle not airborne";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (stop_commanding_.load()) {
    response->success = false;
    response->message = "Waypoint not set, vehicle is under manual control";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!takeoff_completed_.load()) {
    response->success = false;
    response->message = "Waypoint not set, vehicle not flying normally";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!stopPreviousMission()) {
    response->success = false;
    response->message = "Waypoint not set, previous mission cannot be aborted";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  response->message = "Waypoint set";
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());

  gps_waypoint_t w;
  w.latitude  = request->goal[0];
  w.longitude = request->goal[1];
  w.altitude  = request->goal[2];
  w.yaw       = request->goal[3];
  {
    std::scoped_lock lock(waypoint_buffer_mutex_, coord_transform_mutex_);
    waypoint_buffer_.push_back(globalToLocal(coord_transform_, w));
  }
  motion_started_.store(true);
  return true;
}
//}

/* gpsPathCallback //{ */
bool ControlInterface::gpsPathCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Waypoints not set, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gps_origin_set_.load()) {
    response->success = false;
    response->message = "Waypoints not set, GPS origin not set";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (stop_commanding_.load()) {
    response->success = false;
    response->message = "Waypoints not set, vehicle is under manual control";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!takeoff_completed_.load()) {
    response->success = false;
    response->message = "Waypoints not set, vehicle not flying normally";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (request->path.poses.size() < 1) {
    response->success = false;
    response->message = "Waypoints not set, request is empty";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!stopPreviousMission()) {
    response->success = false;
    response->message = "Waypoints not set, previous mission cannot be aborted";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  {
    std::scoped_lock lock(waypoint_buffer_mutex_, coord_transform_mutex_);

    RCLCPP_INFO(this->get_logger(), "[%s]: Got %d waypoints", this->get_name(), request->path.poses.size());
    for (size_t i = 0; i < request->path.poses.size(); i++) {
      gps_waypoint_t w;
      w.latitude  = request->path.poses[i].pose.position.x;
      w.longitude = request->path.poses[i].pose.position.y;
      w.altitude  = request->path.poses[i].pose.position.z;
      w.yaw       = getYaw(request->path.poses[i].pose.orientation);
      waypoint_buffer_.push_back(globalToLocal(coord_transform_, w));
    }
  }
  motion_started_.store(true);
  response->success = true;
  response->message = "Waypoints set";
  return true;
}
//}

/* waypointToLocalCallback (gps -> local frame) //{ */
bool ControlInterface::waypointToLocalCallback(const std::shared_ptr<fog_msgs::srv::WaypointToLocal::Request> request,
                                               std::shared_ptr<fog_msgs::srv::WaypointToLocal::Response>      response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Cannot transform coordinates, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gps_origin_set_.load()) {
    response->success = false;
    response->message = "Cannot transform coordinates, missing GPS origin";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  gps_waypoint_t global;
  global.latitude  = request->latitude_deg;
  global.longitude = request->longitude_deg;
  global.altitude  = request->relative_altitude_m;
  global.yaw       = request->yaw;

  {
    std::scoped_lock lock(coord_transform_mutex_);
    local_waypoint_t local = globalToLocal(coord_transform_, global);

    response->local_x = local.x;
    response->local_y = local.y;
    response->local_z = local.z;
    response->yaw     = local.yaw;
  }

  std::stringstream ss;
  ss << "Transformed GPS [" << request->latitude_deg << ", " << request->longitude_deg << ", " << request->relative_altitude_m << "] into local: ["
     << response->local_x << ", " << response->local_y << ", " << response->local_z << "]";
  response->message = ss.str();
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
  return true;
}
//}

/* pathToLocalCallback (gps -> local frame) //{ */
bool ControlInterface::pathToLocalCallback(const std::shared_ptr<fog_msgs::srv::PathToLocal::Request> request,
                                           std::shared_ptr<fog_msgs::srv::PathToLocal::Response>      response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Cannot transform coordinates, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gps_origin_set_.load()) {
    response->success = false;
    response->message = "Cannot transform coordinates, GPS origin not set";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  nav_msgs::msg::Path local_path;

  for (auto &pose : request->path.poses) {
    geometry_msgs::msg::Point p_in = pose.pose.position;

    gps_waypoint_t global;
    global.latitude  = p_in.x;
    global.longitude = p_in.y;
    global.altitude  = p_in.z;

    geometry_msgs::msg::Point p_out;
    {
      std::scoped_lock lock(coord_transform_mutex_);
      local_waypoint_t local = globalToLocal(coord_transform_, global);

      p_out.x = local.x;
      p_out.y = local.y;
      p_out.z = local.z;
    }

    std::stringstream ss;
    ss << "Transformed GPS [" << global.latitude << ", " << global.longitude << "] into local: [" << p_out.x << ", " << p_out.y << "]";
    response->message = ss.str();
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());

    geometry_msgs::msg::PoseStamped p_stamped;
    p_stamped.pose.position    = p_out;
    p_stamped.pose.orientation = pose.pose.orientation;
    local_path.poses.push_back(p_stamped);
    local_path.header.frame_id = "local";
    local_path.header.stamp    = this->get_clock()->now();
  }
  std::stringstream ss;
  ss << "Transformed " << request->path.poses.size() << " GPS poses into " << response->path.poses.size() << " local poses";
  response->path    = local_path;
  response->success = true;
  response->message = ss.str();
  RCLCPP_INFO(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());

  return true;
}
//}

/* setPx4ParamIntCallback //{ */
bool ControlInterface::setPx4ParamIntCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Request> request,
                                              std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Response>                       response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Parameter cannot be set, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  auto result = param_->set_param_int(request->param_name, request->value);

  if (result == mavsdk::Param::Result::Success) {
    response->message    = "Parameter set successfully";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = true;
    RCLCPP_INFO(this->get_logger(), "[ControlInterface]: PX4 parameter %s successfully set to %d", request->param_name.c_str(), request->value);
  } else if (result == mavsdk::Param::Result::Unknown) {
    response->message    = "Parameter did not set - unknown error";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter set uknown error");
  } else if (result == mavsdk::Param::Result::Timeout) {
    response->message    = "Parameter did not set - time out";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter set request time out");
  } else if (result == mavsdk::Param::Result::ConnectionError) {
    response->message    = "Parameter did not set - connection error";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter set request connection error");
  } else if (result == mavsdk::Param::Result::WrongType) {
    response->message    = "Parameter did not set - request wrong type";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter set request wrong type");
  } else if (result == mavsdk::Param::Result::ParamNameTooLong) {
    response->message    = "Parameter did not set - param name too long";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter set request param name too long");
  }

  return true;
}
//}

/* getPx4ParamIntCallback //{ */
bool ControlInterface::getPx4ParamIntCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::GetPx4ParamInt::Request> request,
                                              std::shared_ptr<fog_msgs::srv::GetPx4ParamInt::Response>                       response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Parameter cannot be get, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  auto result = param_->get_param_int(request->param_name);

  if (result.first == mavsdk::Param::Result::Success) {
    response->message    = "Parameter get successfully";
    response->value      = result.second;
    response->param_name = request->param_name;
    response->success    = true;

    RCLCPP_INFO(this->get_logger(), "[ControlInterface]: PX4 parameter %s successfully get with value %d", request->param_name.c_str(), response->value);
  } else if (result.first == mavsdk::Param::Result::Unknown) {
    response->message    = "Did not get the parameter - unknown error";
    response->param_name = request->param_name;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter get uknown error");
  } else if (result.first == mavsdk::Param::Result::Timeout) {
    response->message    = "Did not get the parameter - time out";
    response->param_name = request->param_name;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter get request time out");
  } else if (result.first == mavsdk::Param::Result::ConnectionError) {
    response->message    = "Did not get the parameter - connection error";
    response->param_name = request->param_name;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter get request connection error");
  } else if (result.first == mavsdk::Param::Result::WrongType) {
    response->message    = "Did not get the parameter - request wrong type";
    response->param_name = request->param_name;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter get request wrong type");
  } else if (result.first == mavsdk::Param::Result::ParamNameTooLong) {
    response->message    = "Did not get the parameter - param name too long";
    response->param_name = request->param_name;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter get request param name too long");
  }

  return true;
}
//}

/* setPx4ParamFloatCallback //{ */
bool ControlInterface::setPx4ParamFloatCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::SetPx4ParamFloat::Request> request,
                                                std::shared_ptr<fog_msgs::srv::SetPx4ParamFloat::Response>                       response) {

  if (!is_initialized_.load()) {
    response->success = false;
    response->message = "Parameter cannot be set, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  auto result = param_->set_param_float(request->param_name, request->value);

  if (result == mavsdk::Param::Result::Success) {
    response->message    = "Parameter set successfully";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = true;
    RCLCPP_INFO(this->get_logger(), "[ControlInterface]: PX4 parameter %s successfully set to %f", request->param_name.c_str(), request->value);
  } else if (result == mavsdk::Param::Result::Unknown) {
    response->message    = "Parameter did not set - unknown error";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter set uknown error");
  } else if (result == mavsdk::Param::Result::Timeout) {
    response->message    = "Parameter did not set - time out";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter set request time out");
  } else if (result == mavsdk::Param::Result::ConnectionError) {
    response->message    = "Parameter did not set - connection error";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter set request connection error");
  } else if (result == mavsdk::Param::Result::WrongType) {
    response->message    = "Parameter did not set - request wrong type";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter set request wrong type");
  } else if (result == mavsdk::Param::Result::ParamNameTooLong) {
    response->message    = "Parameter did not set - param name too long";
    response->param_name = request->param_name;
    response->value      = request->value;
    response->success    = false;
    RCLCPP_ERROR(this->get_logger(), "[ControlInterface]: PX4 parameter set request param name too long");
  }

  return true;
}
//}

/* gpsOriginCallback //{ */
bool ControlInterface::gpsOriginCallback(rclcpp::Client<fog_msgs::srv::GetOrigin>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::GetOrigin::Response> result = future.get();
  if (result->success) {
    mavsdk::geometry::CoordinateTransformation::GlobalCoordinate ref;
    ref.latitude_deg  = result->latitude;
    ref.longitude_deg = result->longitude;
    coord_transform_  = std::make_shared<mavsdk::geometry::CoordinateTransformation>(mavsdk::geometry::CoordinateTransformation(ref));

    RCLCPP_INFO(this->get_logger(), "[%s]: GPS origin set! Lat: %.3f, Lon: %.3f", this->get_name(), ref.latitude_deg, ref.longitude_deg);
    gps_origin_set_.store(true);
  } else {

    auto &clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "[%s]: Waiting for GPS origin.", this->get_name());
    gps_origin_called_.store(false);
    return false;
  }
  gps_origin_called_.store(false);
  return true;
}
//}

/* odomAvailableCallback //{ */
bool ControlInterface::odomAvailableCallback(rclcpp::Client<fog_msgs::srv::GetBool>::SharedFuture future) {
  std::shared_ptr<fog_msgs::srv::GetBool::Response> result = future.get();
  if (result->value) {
    RCLCPP_INFO(this->get_logger(), "[%s]: Odometry available!", this->get_name());
    getting_odom_.store(true);
  } else {
    auto &clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(this->get_logger(), clk, 1000, "[%s]: Waiting for Odometry", this->get_name());
    getting_odom_called_.store(false);
    return false;
  }
  getting_odom_called_.store(false);
  return true;
}
//}

/* /1* homeCallback //{ *1/ */
/* void ControlInterface::homeCallback(const mavsdk::Telemetry::Position home_position) { */
/*   RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting home position from telemetry!", this->get_name()); */

/*   if (!gps_origin_set_.load()) { */
/*     mavsdk::geometry::CoordinateTransformation::GlobalCoordinate ref; */
/*     ref.latitude_deg  = home_position.latitude_deg; */
/*     ref.longitude_deg = home_position.longitude_deg; */
/*     coord_transform_  = std::make_shared<mavsdk::geometry::CoordinateTransformation>(mavsdk::geometry::CoordinateTransformation(ref)); */

/*     RCLCPP_INFO(this->get_logger(), "[%s]: GPS origin set! Lat: %.3f, Lon: %.3f", this->get_name(), ref.latitude_deg, ref.longitude_deg); */
/*     gps_origin_set_.store(true); */
/*   } */
/* } */
/* //} */

/* controlRoutine //{ */
void ControlInterface::controlRoutine(void) {

  if (is_initialized_.load()) {
    publishDiagnostics();
    publishDesiredPose();

    if (gps_origin_set_.load() && getting_odom_.load()) {

      RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: CONTROL INTERFACE IS READY", this->get_name());

      if (!armed_.load()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Vehicle not armed", this->get_name());
        return;
      }

      if (landed_.load()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Vehicle not airborne", this->get_name());
        return;
      }

      if (stop_commanding_.load()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Control action prevented by an external trigger", this->get_name());
        return;
      }

      /* handle motion //{ */
      if (motion_started_.load()) {

        {
          std::scoped_lock lock(waypoint_buffer_mutex_, mission_mutex_);

          // create a new mission plan if there are unused points in buffer
          if (waypoint_buffer_.size() > 0 && mission_finished_.load()) {
            publishDebugMarkers();
            RCLCPP_INFO(this->get_logger(), "[%s]: Waypoints to be visited: %ld", this->get_name(), waypoint_buffer_.size());
            mission_->pause_mission();
            mission_plan_.mission_items.clear();

            addToMission(waypoint_buffer_.front());
            desired_pose_ = Eigen::Vector4d(waypoint_buffer_.front().x, waypoint_buffer_.front().y, waypoint_buffer_.front().z, waypoint_buffer_.front().yaw);
            waypoint_buffer_.pop_front();

            start_mission_.store(true);
          }
        }

        {
          std::scoped_lock lock(mission_mutex_);
          // upload and execute new mission
          if (start_mission_.load() && mission_plan_.mission_items.size() > 0) {
            bool success = uploadMission() && startMission();
            if (success) {
              mission_finished_.store(false);
              start_mission_.store(false);
            }
          }
        }

        // stop if final goal is reached
        if (mission_finished_.load()) {
          RCLCPP_INFO(this->get_logger(), "[%s]: All waypoints have been visited", this->get_name());
          motion_started_.store(false);
        }
      }
      //}

    } else {
      // Check the availability of the GPS origin
      if (!gps_origin_set_.load()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: GPS origin not set", this->get_name());
      }
      // Check the availability of the Odometry
      if (!getting_odom_.load() && !getting_odom_called_.load()) {
        getting_odom_called_.store(true);
        auto request     = std::make_shared<fog_msgs::srv::GetBool::Request>();
        auto call_result = getting_odom_client_->async_send_request(request, std::bind(&ControlInterface::odomAvailableCallback, this, std::placeholders::_1));
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Odometry availability service called", this->get_name());
      }
      printSensorsStatus();
    }
  }
}
//}

/* printSensorsStatus //{ */
void ControlInterface::printSensorsStatus() {

  // TODO FIXME
  // leave the checks up to odometry
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: GPS origin set:%s", this->get_name(), gps_origin_set_.load() ? "TRUE" : "FALSE");
}
//}

/* publishDiagnostics //{ */
void ControlInterface::publishDiagnostics() {
  fog_msgs::msg::ControlInterfaceDiagnostics msg;
  msg.header.stamp         = this->get_clock()->now();
  msg.header.frame_id      = world_frame_;
  msg.armed                = armed_.load();
  msg.airborne             = !landed_.load() && takeoff_completed_;
  msg.moving               = motion_started_.load();
  msg.mission_finished     = mission_finished_.load();
  msg.getting_control_mode = getting_control_mode_.load();
  msg.getting_land_sensor  = getting_landed_info_.load();
  msg.gps_origin_set       = gps_origin_set_.load();
  msg.getting_odom         = getting_odom_.load();
  msg.manual_control       = stop_commanding_.load();

  {
    std::scoped_lock lock(waypoint_buffer_mutex_);
    msg.buffered_mission_items = waypoint_buffer_.size();
  }

  diagnostics_publisher_->publish(msg);
}
//}

/* takeoff //{ */
bool ControlInterface::takeoff() {
  auto result = action_->set_takeoff_altitude(takeoff_height_);
  if (result != mavsdk::Action::Result::Success) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Failed to set takeoff height %.2f", this->get_name(), takeoff_height_);
    return false;
  }

  if (reset_octomap_before_takeoff_) {
    auto reset_srv   = std::make_shared<std_srvs::srv::Empty::Request>();
    auto call_result = octomap_reset_client_->async_send_request(reset_srv);
    RCLCPP_INFO(this->get_logger(), "[%s]: Resetting octomap server", this->get_name());
  }

  action_->takeoff();
  if (result != mavsdk::Action::Result::Success) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Takeoff failed", this->get_name());
    return false;
  }


  if (pos_samples_.size() < takeoff_position_samples_) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Takeoff rejected. Need %ld odometry samples, only have %ld", this->get_name(), takeoff_position_samples_,
                pos_samples_.size());
    return false;
  }

  local_waypoint_t current_goal;

  // averaging desired takeoff position
  current_goal.x = 0;
  current_goal.y = 0;

  for (const auto &p : pos_samples_) {
    current_goal.x += p[0];
    current_goal.y += p[1];
  }
  current_goal.x /= pos_samples_.size();
  current_goal.y /= pos_samples_.size();

  current_goal.z   = takeoff_height_;
  current_goal.yaw = getYaw(ori_);
  desired_pose_    = Eigen::Vector4d(current_goal.x, current_goal.y, current_goal.z, current_goal.yaw);

  takeoff_called_.store(true);
  RCLCPP_INFO(this->get_logger(), "[%s]: Taking off", this->get_name());
  return true;
}
//}

/* land //{ */
bool ControlInterface::land() {

  RCLCPP_INFO(this->get_logger(), "[%s]: Landing called. Stop commanding", this->get_name());
  stop_commanding_.store(true);
  takeoff_completed_.store(false);

  auto result = action_->land();
  if (result != mavsdk::Action::Result::Success) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Landing failed", this->get_name());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Landing", this->get_name());
  return true;
}
//}

/* startMission //{ */
bool ControlInterface::startMission() {

  if (stop_commanding_.load()) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Mission start prevented by external trigger", this->get_name());
    return false;
  }

  auto result = mission_->start_mission();
  if (result != mavsdk::Mission::Result::Success) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Mission start rejected", this->get_name());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Mission started", this->get_name());
  return true;
}
//}

/* uploadMission //{ */
bool ControlInterface::uploadMission() {

  if (stop_commanding_.load()) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Mission upload prevented by external trigger", this->get_name());
    return false;
  }

  auto clear_result = mission_->clear_mission();
  if (clear_result != mavsdk::Mission::Result::Success) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Mission upload failed. Could not clear previous mission", this->get_name());
    return false;
  }

  auto result = mission_->upload_mission(mission_plan_);
  if (result != mavsdk::Mission::Result::Success) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Mission upload failed", this->get_name());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: Mission uploaded", this->get_name());

  return true;
}
//}

/* stopPreviousMission //{ */
bool ControlInterface::stopPreviousMission() {

  if (!motion_started_.load()) {
    return true;
  }

  motion_started_.store(false);
  start_mission_.store(false);
  mission_finished_.store(true);

  auto result = mission_->clear_mission();

  {
    std::scoped_lock lock(waypoint_buffer_mutex_, mission_mutex_);
    mission_plan_.mission_items.clear();
    waypoint_buffer_.clear();

    if (result != mavsdk::Mission::Result::Success || mission_plan_.mission_items.size() > 0) {
      RCLCPP_ERROR(this->get_logger(), "[%s]: Previous mission cannot be stopped", this->get_name());
      return false;
    }
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Previous mission stopped", this->get_name());
  return true;
}
//}

/* addToMission //{ */
void ControlInterface::addToMission(local_waypoint_t w) {

  // apply home offset correction
  w.x -= home_position_offset_.y();
  w.y -= home_position_offset_.x();

  mavsdk::Mission::MissionItem item;
  gps_waypoint_t               global = localToGlobal(coord_transform_, w);
  item.latitude_deg                   = global.latitude;
  item.longitude_deg                  = global.longitude;
  item.relative_altitude_m            = global.altitude;
  item.yaw_deg                        = -radToDeg(global.yaw + yaw_offset_correction_);
  item.speed_m_s                      = target_velocity_;  // NAN = use default values. This does NOT
                                                           // limit vehicle max speed
  item.is_fly_through          = true;
  item.gimbal_pitch_deg        = 0.0f;
  item.gimbal_yaw_deg          = 0.0f;
  item.camera_action           = mavsdk::Mission::MissionItem::CameraAction::None;
  item.loiter_time_s           = waypoint_loiter_time_;
  item.camera_photo_interval_s = 0.0f;
  item.acceptance_radius_m     = waypoint_acceptance_radius_;

  mission_plan_.mission_items.push_back(item);

  RCLCPP_INFO(this->get_logger(), "[%s]: Added waypoint LOCAL: [%.2f, %.2f, %.2f, %.2f]", this->get_name(), w.x, w.y, w.z, w.yaw);
  RCLCPP_INFO(this->get_logger(), "[%s]: GLOBAL: [%.2f, %.2f, %.2f, %.2f]", this->get_name(), item.latitude_deg, item.longitude_deg, item.relative_altitude_m,
              item.yaw_deg);
}
//}

/* publishDesiredPose //{ */
void ControlInterface::publishDesiredPose() {
  /* if (desired_pose_.z() < 0.5) { */
  /*   return; */
  /* } */
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp     = this->get_clock()->now();
  msg.header.frame_id  = world_frame_;
  msg.pose.position.x  = desired_pose_.x();
  msg.pose.position.y  = desired_pose_.y();
  msg.pose.position.z  = desired_pose_.z();
  Eigen::Quaterniond q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(desired_pose_.w(), Eigen::Vector3d::UnitZ());
  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  desired_pose_publisher_->publish(msg);
}
//}

/* publishDebugMarkers //{ */
void ControlInterface::publishDebugMarkers() {
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp    = this->get_clock()->now();
  msg.header.frame_id = world_frame_;

  for (auto &w : waypoint_buffer_) {
    geometry_msgs::msg::Pose p;
    p.position.x = w.x;
    p.position.y = w.y;
    p.position.z = w.z;
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(w.yaw, Eigen::Vector3d::UnitZ());
    p.orientation.w = q.w();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    msg.poses.push_back(p);
  }
  waypoint_marker_publisher_->publish(msg);
}
//}

/* generateColor//{ */
std_msgs::msg::ColorRGBA ControlInterface::generateColor(const double r, const double g, const double b, const double a) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}
//}

/* parse_param //{ */
template <class T>
bool ControlInterface::parse_param(const std::string &param_name, T &param_dest) {
  this->declare_parameter(param_name);
  if (!this->get_parameter(param_name, param_dest)) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Could not load param '%s'", this->get_name(), param_name.c_str());
    return false;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << this->get_name() << "]: Loaded '" << param_name << "' = '" << param_dest << "'");
  }
  return true;
}
//}

//}

}  // namespace control_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(control_interface::ControlInterface)
