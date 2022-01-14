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
#include <mutex>
#include <fstream>
#include <boost/circular_buffer.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/mission_result.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
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
#include <visualization_msgs/msg/marker_array.hpp>

//}

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
// TODO: Is this really what we want? Or do we want heading (angle from the x-axis in the XY plane) since that's what's published in the debugs?
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

/* add_reason_if helper string function //{ */

void add_reason_if(const std::string& reason, const bool condition, std::string& to_str)
{
  if (condition)
  {
    if (to_str.empty())
      to_str = reason;
    else
      to_str = to_str + ", " + reason;
  }
}

//}

std::string to_string(const mavsdk::Action::Result result);
std::string to_string(const mavsdk::Mission::Result result);
std::string to_string(const mavsdk::Param::Result result);
std::string to_string(const mavsdk::ConnectionResult result);
std::string to_string(const mavsdk::Telemetry::FlightMode mode);
std::string to_string(const mavsdk::Telemetry::LandedState land_state);

/* helper class scope_timer //{ */

class scope_timer
{
  public:
    scope_timer(
        const bool enable, // whether to print anything - if false, nothing will be done
        const std::string& label, // label of this timer used to uniquely identify it and for printing
        const rclcpp::Duration& throttle = {0, 0}, // prints will not be output with a shorter period than this
        const rclcpp::Duration& min_dur = {0, 0}, // shorter durations will be ignored
        rclcpp::Clock::SharedPtr clock_ptr = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME) // which clock to use
      )
      : enable_(enable), label_(label), throttle_(throttle), min_dur_(min_dur), start_time_(clock_ptr->now()), clock_ptr_(clock_ptr)
    {}

    ~scope_timer()
    {
      if (!enable_)
        return;

      const rclcpp::Time end_time = clock_ptr_->now();
      const rclcpp::Duration dur = end_time - start_time_;
      // ignore shorter durations than min_dur_
      if (dur < min_dur_)
        return;

      // finally, check if it's been long enough from the last print for a new message
      std::scoped_lock lck(last_message_mtx_);
      if (!last_message_.count(label_) || end_time - last_message_.at(label_)  > throttle_)
      {
        std::cout << label_ << " took " << dur.seconds() << "s" << std::endl;
        last_message_.insert_or_assign(label_, end_time);
      }
    }

  private:
    const bool enable_;
    const std::string label_;
    const rclcpp::Duration throttle_;
    const rclcpp::Duration min_dur_;
    const rclcpp::Time start_time_;
    const rclcpp::Clock::SharedPtr clock_ptr_;
    std::mutex last_message_mtx_;
    static std::unordered_map<std::string, rclcpp::Time> last_message_;
};

//}

std::unordered_map<std::string, rclcpp::Time> scope_timer::last_message_ = {};

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
  enum vehicle_state_t
  {
    not_connected,
    not_ready,
    takeoff_ready,
    taking_off,
    autonomous_flight,
    manual_flight,
  } vehicle_state_ = not_connected;
  std::string vehicle_state_str_ = "MavSDK system not connected";

  std::atomic_bool system_connected_ = false; // set to true when MavSDK connection is established
  std::atomic_bool getting_control_mode_ = false; // set to true when a VehicleControlMode is received from pixhawk
  std::atomic_bool getting_odom_ = false;
  std::atomic_bool gps_origin_set_ = false;

  std::atomic_bool manual_override_ = false;

  std::recursive_mutex mission_mutex_;
  std::shared_ptr<mavsdk::Mission> mission_;
  bool mission_finished_flag_; // set to true in the missionResultCallback, this flag is cleared in the main missionControlRoutine
  unsigned mission_last_instance_ = 1;
  enum mission_state_t
  {
    uploading,
    in_progress,
    finished
  } mission_state_ = finished;

  std::mutex mission_progress_mutex_;
  int mission_progress_size_ = 0;
  int mission_progress_current_waypoint_ = 0;

  std::recursive_mutex mission_upload_mutex_;
  int mission_upload_attempts_;
  mavsdk::Mission::MissionPlan  mission_upload_waypoints_; // this buffer is used for repeated attempts at mission uploads
  rclcpp::Time mission_upload_start_time_;
  rclcpp::Time mission_upload_end_time_;
  enum mission_upload_state_t
  {
    started,
    done,
    failed
  } mission_upload_state_ = done;

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

  std::mutex                    waypoint_buffer_mutex_;  // guards the following block of variables
  std::vector<local_waypoint_t> waypoint_buffer_;        // this buffer is used for storing new incoming waypoints (it is moved to the mission_upload_waypoints_ buffer when upload starts)

  Eigen::Vector4d desired_pose_;
  Eigen::Vector3d home_position_offset_ = Eigen::Vector3d(0, 0, 0);

  // use takeoff lat and long to initialize local frame
  std::mutex                                                  coord_transform_mutex_;
  std::shared_ptr<mavsdk::geometry::CoordinateTransformation> coord_transform_;

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
  rclcpp::Duration print_callback_min_dur_ = rclcpp::Duration::from_seconds(0.020);
  int    mavsdk_logging_print_level_        = 1;
  std::string mavsdk_logging_filename_      = {};
  double yaw_offset_correction_             = M_PI / 2;
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr desired_pose_publisher_;  // https://ctu-mrs.github.io/docs/system/relative_commands.html
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   waypoint_publisher_;
  rclcpp::Publisher<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr diagnostics_publisher_;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr  control_mode_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::MissionResult>::SharedPtr       mission_result_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr        home_position_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odometry_subscriber_;

  // callback groups
  // a shared pointer to each callback group has to be saved or the callbacks will never get called
  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  // new callback groups have to be initialized using this function to be saved into callback_groups_
  rclcpp::CallbackGroup::SharedPtr new_cbk_grp();

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

  // subscriber callbacks
  void controlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);
  void missionResultCallback(const px4_msgs::msg::MissionResult::UniquePtr msg);
  void missionProgressCallback(const mavsdk::Mission::MissionProgress& mission_progress);
  bool mavsdkLogCallback(const mavsdk::log::Level level, const std::string& message, const std::string& file, const int line);
  void homePositionCallback(const px4_msgs::msg::HomePosition::UniquePtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg);

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
  bool startMission();
  bool startMissionUpload(const mavsdk::Mission::MissionPlan& mission_plan);
  bool stopMission(std::string& fail_reason_out);

  void publishDebugMarkers();
  void publishDesiredPose();

  // timers
  rclcpp::TimerBase::SharedPtr mission_control_timer_;
  void missionControlRoutine();

  rclcpp::TimerBase::SharedPtr vehicle_state_timer_;
  void vehicleStateRoutine();

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  void diagnosticsRoutine();

  // helper state methods
  void state_mission_finished();
  void state_mission_uploading();
  void state_mission_in_progress();

  void updateVehicleState(bool takeoff_started = false);
  void state_vehicle_not_connected();
  void state_vehicle_not_ready();
  void state_vehicle_takeoff_ready(bool takeoff_started);
  void state_vehicle_taking_off();
  void state_vehicle_autonomous_flight();
  void state_vehicle_manual_flight();
  void state_vehicle_landing();

  // utils
  template <class T>
  bool parse_param(const std::string &param_name, T &param_dest);
  bool parse_param(const std::string& param_name, rclcpp::Duration& param_dest);

  template<typename T>
  bool addWaypoints(const T& path, const bool is_global, std::string& fail_reason_out);
  mavsdk::Mission::MissionItem to_mission_item(const local_waypoint_t& w_in);
  local_waypoint_t to_local_waypoint(const geometry_msgs::msg::PoseStamped& in, const bool is_global);
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
  loaded_successfully &= parse_param("device_url", device_url_);
  loaded_successfully &= parse_param("world_frame", world_frame_);
  //}

  /* parse params from config file //{ */
  loaded_successfully &= parse_param("general.octomap_reset_before_takeoff", octomap_reset_before_takeoff_);
  loaded_successfully &= parse_param("general.octomap_reset_timeout", octomap_reset_timeout_);
  loaded_successfully &= parse_param("general.control_update_rate", control_update_rate_);
  loaded_successfully &= parse_param("general.diagnostics_publish_rate", diagnostics_publish_rate_);
  loaded_successfully &= parse_param("general.print_callback_durations", print_callback_durations_);
  loaded_successfully &= parse_param("general.print_callback_min_dur", print_callback_min_dur_);

  loaded_successfully &= parse_param("takeoff.height", takeoff_height_);
  loaded_successfully &= parse_param("takeoff.position_samples", takeoff_position_samples_);

  loaded_successfully &= parse_param("px4.target_velocity", target_velocity_);
  loaded_successfully &= parse_param("px4.waypoint_loiter_time", waypoint_loiter_time_);
  loaded_successfully &= parse_param("px4.waypoint_acceptance_radius", waypoint_acceptance_radius_);
  loaded_successfully &= parse_param("px4.altitude_acceptance_radius", altitude_acceptance_radius_);

  loaded_successfully &= parse_param("mavsdk.logging_print_level", mavsdk_logging_print_level_);
  loaded_successfully &= parse_param("mavsdk.logging_filename", mavsdk_logging_filename_);
  loaded_successfully &= parse_param("mavsdk.yaw_offset_correction", yaw_offset_correction_);
  loaded_successfully &= parse_param("mavsdk.mission_upload_attempts_threshold", mission_upload_attempts_threshold_);

  if (!loaded_successfully) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  if (control_update_rate_ < 5.0) {
    control_update_rate_ = 5.0;
    RCLCPP_WARN(get_logger(), "Control update rate set too slow. Defaulting to 5 Hz");
  }

  //}

  // | ------------- misc. parameters initialization ------------ |
  desired_pose_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
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

  rclcpp::QoS qos(rclcpp::KeepLast(3));
  // | ------------------ initialize publishers ----------------- |
  desired_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/desired_pose_out", qos);
  waypoint_publisher_     = create_publisher<geometry_msgs::msg::PoseArray>("~/waypoints_out", qos);
  diagnostics_publisher_  = create_publisher<fog_msgs::msg::ControlInterfaceDiagnostics>("~/diagnostics_out", qos);

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
  mission_result_subscriber_ = create_subscription<px4_msgs::msg::MissionResult>("~/mission_result_in",
      rclcpp::SystemDefaultsQoS(), std::bind(&ControlInterface::missionResultCallback, this, _1), subopts);

  subopts.callback_group = new_cbk_grp();
  home_position_subscriber_ = create_subscription<px4_msgs::msg::HomePosition>("~/home_position_in",
      rclcpp::SystemDefaultsQoS(), std::bind(&ControlInterface::homePositionCallback, this, _1), subopts);

  subopts.callback_group = new_cbk_grp();
  odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>("~/local_odom_in",
      rclcpp::SystemDefaultsQoS(), std::bind(&ControlInterface::odometryCallback, this, _1), subopts);

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

  mission_control_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / control_update_rate_),
      std::bind(&ControlInterface::missionControlRoutine, this), new_cbk_grp());

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
  scope_timer tim(print_callback_durations_, "controlModeCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(telem_mutex_, mission_mutex_, mission_upload_mutex_, waypoint_buffer_mutex_);

  if (!system_connected_)
    return;

  getting_control_mode_ = true;

  // evaluate manual control switching (can only be switched to off if vehicle is landed and disarmed)
  if (!manual_override_ && msg->flag_control_manual_enabled)
  {
    RCLCPP_INFO(get_logger(), "Control flag switched to manual. Stopping and clearing mission");
    manual_override_ = true;
    std::string fail_reason;
    if (!stopMission(fail_reason))
      RCLCPP_ERROR_STREAM(get_logger(), "Previous mission cannot be stopped (" << fail_reason << "). Manual landing required");
    return;
  }

  if (manual_override_ && !msg->flag_control_manual_enabled)
  {
    const auto land_state = telem_->landed_state();
    const bool on_ground_disarmed = !msg->flag_armed && land_state == mavsdk::Telemetry::LandedState::OnGround;
    const bool taking_off = land_state == mavsdk::Telemetry::LandedState::TakingOff;
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
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "NOT enabling automatic control: neither taking off nor landed and disarmed (" << to_string(land_state) << ")");
  }
}
//}

/* odometryCallback //{ */
void ControlInterface::odometryCallback(const nav_msgs::msg::Odometry::UniquePtr msg)
{
  scope_timer tim(print_callback_durations_, "odometryCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(pose_mutex_);

  getting_odom_ = true;
  RCLCPP_INFO_ONCE(get_logger(), "Getting odometry");

  // update the current position and orientation of the vehicle
  pose_pos_.x() = msg->pose.pose.position.x;
  pose_pos_.y() = msg->pose.pose.position.y;
  pose_pos_.z() = msg->pose.pose.position.z;
  pose_ori_.setX(msg->pose.pose.orientation.x);
  pose_ori_.setY(msg->pose.pose.orientation.y);
  pose_ori_.setZ(msg->pose.pose.orientation.z);
  pose_ori_.setW(msg->pose.pose.orientation.w);

  // add the current position to the pose samples for takeoff position estimation
  pose_takeoff_samples_.push_back(pose_pos_);
}
//}

/* homePositionCallback //{ */
void ControlInterface::homePositionCallback(const px4_msgs::msg::HomePosition::UniquePtr msg)
{
  scope_timer tim(print_callback_durations_, "homePositionCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(coord_transform_mutex_);

  mavsdk::geometry::CoordinateTransformation::GlobalCoordinate ref;
  ref.latitude_deg  = msg->lat;
  ref.longitude_deg = msg->lon;

  coord_transform_ = std::make_shared<mavsdk::geometry::CoordinateTransformation>(mavsdk::geometry::CoordinateTransformation(ref));

  RCLCPP_INFO(get_logger(), "GPS origin set! Lat: %.6f, Lon: %.6f", ref.latitude_deg, ref.longitude_deg);

  home_position_offset_ = Eigen::Vector3d(msg->y, msg->x, -msg->z);
  RCLCPP_INFO(get_logger(), "Home position offset (local): %.2f, %.2f, %.2f", home_position_offset_.x(),
              home_position_offset_.y(), home_position_offset_.z());

  gps_origin_set_ = true;
}
//}

/* missionResultCallback //{ */
void ControlInterface::missionResultCallback(const px4_msgs::msg::MissionResult::UniquePtr msg)
{
  scope_timer tim(print_callback_durations_, "missionResultCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(mission_mutex_);

  // check if a mission is currently in-progress and we got an indication that it is finished
  if (mission_state_ == mission_state_t::in_progress && msg->finished && msg->instance_count != mission_last_instance_)
  {
    RCLCPP_INFO(get_logger(), "Mission #%u finished by callback", msg->instance_count);
    mission_finished_flag_ = true; // set the appropriate flag to signal the state machine to change state
    mission_last_instance_ = msg->instance_count;
  }
}
//}

/* missionProgressCallback //{ */
void ControlInterface::missionProgressCallback(const mavsdk::Mission::MissionProgress& mission_progress)
{
  // spawn a new thread to avoid blocking in the MavSDK
  // callback which will eventually cause a deadlock
  // of the MavSDK processing thread
  std::thread([this, &mission_progress]
  {
    scope_timer tim(print_callback_durations_, "missionProgressCallback", print_callback_throttle_, print_callback_min_dur_);
    std::scoped_lock lck(mission_progress_mutex_);
    mission_progress_size_ = mission_progress.total;
    mission_progress_current_waypoint_ = mission_progress.current;

    const float percent = mission_progress.total == 0 ? 100 : mission_progress.current/float(mission_progress.total)*100.0f;
    if (mission_progress_current_waypoint_ == -1)
      RCLCPP_INFO(get_logger(), "Current mission cancelled (waypoint %d/%d).", mission_progress.current, mission_progress.total);
    else if (mission_progress_current_waypoint_ == 0)
      RCLCPP_INFO(get_logger(), "Current mission not started yet (waypoint %d/%d).", mission_progress.current, mission_progress.total);
    else
      RCLCPP_INFO(get_logger(), "Current mission waypoint: %d/%d (%.1f%%).", mission_progress.current, mission_progress.total, percent);
  }).detach();
}
//}

/* mavsdkLogCallback() method //{ */
bool ControlInterface::mavsdkLogCallback(const mavsdk::log::Level level, const std::string& message, const std::string& file, const int line)
{
  scope_timer tim(print_callback_durations_, "mavsdkLogCallback", print_callback_throttle_, print_callback_min_dur_);
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
  scope_timer tim(print_callback_durations_, "takeoffCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(state_mutex_, telem_mutex_, action_mutex_, pose_mutex_, waypoint_buffer_mutex_);

  if (vehicle_state_ != vehicle_state_t::takeoff_ready)
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
  updateVehicleState(true); // update the vehicle state now as it should change
  return true;
}
//}

/* landCallback //{ */
bool ControlInterface::landCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response>                       response)
{
  scope_timer tim(print_callback_durations_, "landCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(state_mutex_, mission_mutex_, mission_upload_mutex_, waypoint_buffer_mutex_, action_mutex_);

  std::string fail_reason;
  if (!stopMission(fail_reason))
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
  updateVehicleState(); // update the vehicle state now as it should change
  return true;
}
//}

/* armingCallback //{ */
bool ControlInterface::armingCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response>                       response)
{
  scope_timer tim(print_callback_durations_, "armingCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(state_mutex_, action_mutex_);

  if (!system_connected_)
  {
    response->success = false;
    response->message = "Arming rejected, not initialized";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return true;
  }

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
      response->message = "Vehicle armed";
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
      response->message = "Vehicle disarmed";
      response->success = true;
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return true;
    }
  }
}
//}

// | ----------------- Waypoint/path callbacks ---------------- |

/* addWaypoints() method //{ */
template<typename T>
bool ControlInterface::addWaypoints(const T& path, const bool is_global, std::string& fail_reason_out)
{
  scope_timer tim(print_callback_durations_, "addWaypoints", print_callback_throttle_, print_callback_min_dur_);
  // check that we are in a state in which we can add waypoints to the buffer
  if (vehicle_state_ != vehicle_state_t::autonomous_flight)
  {
    fail_reason_out = "not flying! Current state is: " + vehicle_state_str_;
    return false;
  }

  // stop the current mission (if any)
  std::string fail_reason;
  if (!stopMission(fail_reason))
  {
    fail_reason_out = "previous mission cannot be aborted (" + fail_reason + ")";
    return false;
  }

  // finally, add the waypoints to the buffer
  for (const auto& pose : path)
    waypoint_buffer_.push_back(to_local_waypoint(pose, is_global));

  return true;
}
//}

/* localWaypointCallback //{ */
bool ControlInterface::localWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                             std::shared_ptr<fog_msgs::srv::Vec4::Response>      response)
{
  scope_timer tim(print_callback_durations_, "localWaypointCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(state_mutex_, waypoint_buffer_mutex_, mission_mutex_, mission_upload_mutex_);

  // convert the single waypoint to a path containing a single point
  std::vector<std::vector<double>> path {request->goal};

  // check that we are in a state in which we can add waypoints to the buffer
  std::string reason;
  if (!addWaypoints(path, false, reason))
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
  scope_timer tim(print_callback_durations_, "localPathCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(state_mutex_, waypoint_buffer_mutex_, mission_mutex_, mission_upload_mutex_);

  // check that we are in a state in which we can add waypoints to the buffer
  std::string reason;
  if (!addWaypoints(request->path.poses, false, reason))
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

/* gpsWaypointCallback //{ */
bool ControlInterface::gpsWaypointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                           std::shared_ptr<fog_msgs::srv::Vec4::Response>      response)
{
  scope_timer tim(print_callback_durations_, "gpsWaypointCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(state_mutex_, waypoint_buffer_mutex_, mission_mutex_, mission_upload_mutex_, coord_transform_mutex_);

  // convert the single waypoint to a path containing a single point
  std::vector<std::vector<double>> path {request->goal};

  // check that we are in a state in which we can add waypoints to the buffer
  std::string reason;
  if (!addWaypoints(path, true, reason))
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
  scope_timer tim(print_callback_durations_, "gpsPathCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(state_mutex_, waypoint_buffer_mutex_, mission_mutex_, mission_upload_mutex_, coord_transform_mutex_);

  // check that we are in a state in which we can add waypoints to the buffer
  std::string reason;
  if (!addWaypoints(request->path.poses, true, reason))
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
  scope_timer tim(print_callback_durations_, "waypointToLocalCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(state_mutex_, coord_transform_mutex_);

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
  response->local_y = local.x;
  response->local_z = local.x;
  response->yaw = local.yaw;

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
  scope_timer tim(print_callback_durations_, "pathToLocalCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(state_mutex_, coord_transform_mutex_);

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
  scope_timer tim(print_callback_durations_, "parametersCallback", print_callback_throttle_, print_callback_min_dur_);
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason     = "";

  for (const auto &param : parameters)
  {
    std::stringstream result_ss;

    /* takeoff_height //{ */
    if (param.get_name() == "takeoff.height") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.5 && param.as_double() < 10) {
          takeoff_height_   = param.as_double();
          result.successful = true;
          RCLCPP_INFO(get_logger(), "Parameter: '%s' set to %1.2f", param.get_name().c_str(), param.as_double());
        } else {
          result_ss << "parameter '" << param.get_name() << "' cannot be set to " << param.as_double() << " because it is not in range <0.5;10>";
          result.reason = result_ss.str();
        }
      } else {
        result_ss << "parameter '" << param.get_name() << "' has to be type DOUBLE";
        result.reason = result_ss.str();
      }
      //}

      /* waypoint_loiter_time //{ */
    } else if (param.get_name() == "px4.waypoint_loiter_time") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0) {
          waypoint_loiter_time_ = param.as_double();
          result.successful     = true;
          RCLCPP_INFO(get_logger(), "Parameter: '%s' set to %1.2f", param.get_name().c_str(), param.as_double());
        } else {
          result_ss << "parameter '" << param.get_name() << "' cannot be set to " << param.as_double() << " because it is a negative value";
          result.reason = result_ss.str();
        }
      } else {
        result_ss << "parameter '" << param.get_name() << "' has to be type DOUBLE";
        result.reason = result_ss.str();
      }
      //}

      /* reset_octomap_before_takeoff //{ */
    } else if (param.get_name() == "general.reset_octomap_before_takeoff") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        octomap_reset_before_takeoff_ = param.as_bool();
        result.successful             = true;
        RCLCPP_INFO(get_logger(), "Parameter: '%s' set to %s", param.get_name().c_str(), param.as_bool() ? "TRUE" : "FALSE");
      } else {
        result_ss << "parameter '" << param.get_name() << "' has to be type BOOL";
        result.reason = result_ss.str();
      }
      //}

    } else {
      result_ss << "parameter '" << param.get_name() << "' cannot be changed dynamically";
      result.reason = result_ss.str();
    }
  }

  if (!result.successful) {
    RCLCPP_WARN(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
  }

  return result;
}
//}

/* setPx4ParamIntCallback //{ */
bool ControlInterface::setPx4ParamIntCallback([[maybe_unused]] const std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Request> request,
                                              std::shared_ptr<fog_msgs::srv::SetPx4ParamInt::Response>                       response)
{
  scope_timer tim(print_callback_durations_, "setPx4ParamIntCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(state_mutex_, param_mutex_);

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
  scope_timer tim(print_callback_durations_, "getPx4ParamIntCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(state_mutex_, param_mutex_);

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
  scope_timer tim(print_callback_durations_, "setPx4ParamFloatCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(state_mutex_, param_mutex_);

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
  scope_timer tim(print_callback_durations_, "getPx4ParamFloatCallback", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(state_mutex_, param_mutex_);

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

/* missionControlRoutine //{ */
void ControlInterface::missionControlRoutine()
{
  scope_timer tim(print_callback_durations_, "missionControlRoutine", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(state_mutex_);

  // check if the vehicle is in a valid state to execute a mission
  if (vehicle_state_ != vehicle_state_t::autonomous_flight)
    return;

  // update the mission (load new waypoints etc.)
  switch (mission_state_)
  {
    case mission_state_t::finished:
      state_mission_finished(); break;
    case mission_state_t::uploading:
      state_mission_uploading(); break;
    case mission_state_t::in_progress:
      state_mission_in_progress(); break;
  }

  publishDesiredPose();
}

// | -------- Implementation of mission control states -------- |

/* ControlInterface::state_mission_finished() //{ */
void ControlInterface::state_mission_finished()
{
  const std::scoped_lock lock(waypoint_buffer_mutex_, mission_mutex_, mission_upload_mutex_);
  // create a new mission plan if there are unused points in the buffer
  if (!waypoint_buffer_.empty())
  {
    publishDebugMarkers();
    RCLCPP_INFO(get_logger(), "New waypoints to be visited: %ld", waypoint_buffer_.size());

    // transform and move the mission waypoints buffer to the mission_upload_waypoints_ buffer with the mavsdk type - we'll attempt to upload the waypoint_upload_buffer_ to pixhawk
    mission_upload_waypoints_.mission_items.clear();
    // transform the points
    for (const auto& pt : waypoint_buffer_)
      mission_upload_waypoints_.mission_items.push_back(to_mission_item(pt));

    // update the current desired_pose_
    const auto& last_wp = waypoint_buffer_.back();
    desired_pose_ = Eigen::Vector4d(last_wp.x, last_wp.y, last_wp.z, last_wp.yaw);

    // clear the waypoint buffer
    waypoint_buffer_.clear();

    // start upload of the new waypoints
    mission_upload_attempts_ = 0;
    startMissionUpload(mission_upload_waypoints_); // this starts the asynchronous upload process
    mission_state_ = mission_state_t::uploading;
  }
}
//}

/* ControlInterface::state_mission_uploading() //{ */
void ControlInterface::state_mission_uploading()
{
  const std::scoped_lock lock(mission_mutex_, mission_upload_mutex_);
  switch (mission_upload_state_)
  {
    // if the mission is being uploaded, just wait for it to either fail or finish
    case mission_upload_state_t::started:
      break;

    // if the mission upload failed, either retry, or fail altogether
    case mission_upload_state_t::failed:
      RCLCPP_WARN(get_logger(), "Mission upload of %ld waypoints failed after %.2fs.", mission_upload_waypoints_.mission_items.size(), (mission_upload_end_time_ - mission_upload_start_time_).seconds());
      if (mission_upload_attempts_ < mission_upload_attempts_threshold_)
      {
        RCLCPP_INFO(get_logger(), "Retrying upload of %ld waypoints", mission_upload_waypoints_.mission_items.size());
        mission_upload_attempts_++;
        startMissionUpload(mission_upload_waypoints_); // this starts the asynchronous upload process
      }
      else
      {
        mission_state_ = mission_state_t::finished;
        RCLCPP_WARN(get_logger(), "Mission upload failed too many times. Scrapping mission.");
      }
      break;

    case mission_upload_state_t::done:
      RCLCPP_INFO(get_logger(), "Mission upload of %ld waypoints succeeded after %.2fs.", mission_upload_waypoints_.mission_items.size(), (mission_upload_end_time_ - mission_upload_start_time_).seconds());
      // clear the mission finish flag and start the uploaded mission
      mission_finished_flag_ = false;
      if (startMission())
        mission_state_ = mission_state_t::in_progress;
      break;
  }
}
//}

/* ControlInterface::state_mission_in_progress() //{ */
void ControlInterface::state_mission_in_progress()
{
  const std::scoped_lock lock(mission_mutex_);
  // stop if final goal is reached
  if (mission_finished_flag_) // this flag is set from the missionResultCallback
  {
    RCLCPP_INFO(get_logger(), "All waypoints have been visited");
    mission_state_ = mission_state_t::finished;
  }
}
//}
//}

/*   diagnosticsRoutine(); //{ */
void ControlInterface::diagnosticsRoutine()
{
  scope_timer tim(print_callback_durations_, "diagnosticsRoutine", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lock(state_mutex_, mission_mutex_, mission_progress_mutex_, telem_mutex_);
  // publish some diags
  publishDiagnostics();
}
//}

/* vehicleStateRoutine //{ */
void ControlInterface::vehicleStateRoutine()
{
  scope_timer tim(print_callback_durations_, "vehicleStateRoutine", print_callback_throttle_, print_callback_min_dur_);
  std::scoped_lock lck(state_mutex_);

  // some debugging of pixhawk states
  /* { */
  /*   std::scoped_lock lck(telem_mutex_); */
  /*   if (telem_) */
  /*   { */
  /*     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Telem FlightMode: %s", to_string(telem_->flight_mode()).c_str()); */
  /*     RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Telem LandedState: %s", to_string(telem_->landed_state()).c_str()); */
  /*   } */
  /* } */

  const auto prev_state = vehicle_state_;
  updateVehicleState();
  if (prev_state != vehicle_state_)
  {
    std::scoped_lock lock(mission_mutex_, mission_progress_mutex_, telem_mutex_);
    publishDiagnostics();
  }
}
//}

/* updateVehicleState //{ */

/* the updateVehicleState() method //{ */
void ControlInterface::updateVehicleState(const bool takeoff_started)
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
      state_vehicle_autonomous_flight(); break;
    case vehicle_state_t::manual_flight:
      state_vehicle_manual_flight(); break;
  }
}
//}

/* state_vehicle_not_connected() method //{ */
void ControlInterface::state_vehicle_not_connected()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vehicle state: MavSDK system not connected.");
  vehicle_state_str_ = "MavSDK system not connected";

  std::scoped_lock lck(action_mutex_, mission_mutex_, mission_upload_mutex_, param_mutex_, telem_mutex_);
  const bool succ = connectPixHawk();
  // advance to the next state if connection and initialization was OK
  if (succ)
  {
    std::string reasons;
    if (!stopMission(reasons))
    {
      RCLCPP_WARN_STREAM(get_logger(), "Failed to stop mission before transitioning to not_ready (" << reasons << "). Arming may be dangerous!");
    }
    else
    {
      RCLCPP_INFO(get_logger(), "MavSDK system connected, clearing all missions and switching state to not_ready.");
      vehicle_state_ = vehicle_state_t::not_ready;
    }
  }
  // otherwise tell the user what we're waiting for
  else
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for MavSDK system connection.");
  }
}
//}

/* state_vehicle_not_ready() method //{ */
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

  std::scoped_lock lck(telem_mutex_);
  const bool armed = telem_->armed();
  const bool healthy = telem_->health_all_ok();
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
    add_reason_if("not healthy", !healthy, reasons);
    add_reason_if("GPS origin not set", !gps_origin_set_, reasons);
    add_reason_if(gps_reason, pose_takeoff_samples_.size() < (size_t)takeoff_position_samples_, reasons);
    add_reason_if("not landed (" + to_string(land_state) + ")", land_state != mavsdk::Telemetry::LandedState::OnGround, reasons);
    vehicle_state_str_ += ", " + reasons;
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Not ready for takeoff: " << reasons);
  }
}
//}

/* state_vehicle_takeoff_ready() method //{ */
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

  std::scoped_lock lck(telem_mutex_, mission_upload_mutex_);
  const bool armed = telem_->armed();
  const bool healthy = telem_->health_all_ok();
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
    add_reason_if("not healthy", !healthy, reasons);
    add_reason_if("not landed (" + to_string(land_state) + ")", land_state != mavsdk::Telemetry::LandedState::OnGround, reasons);
    RCLCPP_INFO_STREAM(get_logger(), "No longer ready for takeoff: " << reasons << ", stopping all missions and switching state to not_ready.");
    if (!stopMission(reasons))
      RCLCPP_WARN_STREAM(get_logger(), "Failed to stop mission before transitioning to not_ready (" << reasons << "). Arming may be dangerous!");
    pose_takeoff_samples_.clear(); // clear the takeoff pose samples to estimate a new one
    vehicle_state_ = vehicle_state_t::not_ready;
    return;
  }
}
//}

/* state_vehicle_taking_off() method //{ */
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

  std::scoped_lock lck(telem_mutex_, mission_upload_mutex_);
  const bool armed = telem_->armed();
  const auto land_state = telem_->landed_state();

  if (!armed)
  {
    RCLCPP_INFO(get_logger(), "Takeoff interrupted with disarm. Stopping all missions and switching state to not_ready.");
    std::string reasons;
    if (!stopMission(reasons))
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
      vehicle_state_ = vehicle_state_t::autonomous_flight;
    }
    return;
  }
}
//}

/* state_vehicle_autonomous_flight() method //{ */
void ControlInterface::state_vehicle_autonomous_flight()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vehicle state: flying autonomously.");
  vehicle_state_str_ = "autonomous flight";

  if (!system_connected_)
  {
    RCLCPP_INFO(get_logger(), "MavSDK system disconnected, switching state to not_connected.");
    vehicle_state_ = vehicle_state_t::not_connected;
    return;
  }

  std::scoped_lock lck(telem_mutex_, mission_upload_mutex_);
  const bool armed = telem_->armed();
  const bool healthy = telem_->health_all_ok();
  const auto land_state = telem_->landed_state();

  if (!armed
   || !healthy
   || land_state != mavsdk::Telemetry::LandedState::InAir)
  {
    std::string reasons;
    add_reason_if("not armed", !armed, reasons);
    add_reason_if("not healthy", !healthy, reasons);
    add_reason_if("not flying (" + to_string(land_state) + ")", land_state != mavsdk::Telemetry::LandedState::InAir, reasons);
    RCLCPP_INFO_STREAM(get_logger(), "Autonomous flight mode ended: " << reasons << ", stopping all missions and switching state to not_ready.");
    if (!stopMission(reasons))
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
void ControlInterface::state_vehicle_manual_flight()
{
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Vehicle state: flying manually.");
  vehicle_state_str_ = "manual flight";

  if (!system_connected_)
  {
    RCLCPP_INFO(get_logger(), "MavSDK system disconnected, switching state to not_connected.");
    vehicle_state_ = vehicle_state_t::not_connected;
    return;
  }

  std::scoped_lock lck(telem_mutex_, mission_upload_mutex_);
  const bool armed = telem_->armed();
  const auto land_state = telem_->landed_state();

  if (!armed
   || land_state != mavsdk::Telemetry::LandedState::InAir)
  {
    std::string reasons;
    add_reason_if("not armed", !armed, reasons);
    add_reason_if("not flying (" + to_string(land_state) + ")", land_state != mavsdk::Telemetry::LandedState::InAir, reasons);
    RCLCPP_INFO_STREAM(get_logger(), "Manual flight mode ended: " << reasons << ", stopping all missions and switching state to not_ready.");
    if (!stopMission(reasons))
      RCLCPP_WARN_STREAM(get_logger(), "Failed to stop mission before transitioning to not_ready (" << reasons << "). Arming may be dangerous!");
    pose_takeoff_samples_.clear(); // clear the takeoff pose samples to estimate a new one
    vehicle_state_ = vehicle_state_t::not_ready;
    return;
  }
}
//}

//}

// | ----------- Action and mission related methods ----------- |

/* connectPixHawk //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// action_mutex_
// mission_mutex_
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
    mission_ = std::make_shared<mavsdk::Mission>(system_);
    param_   = std::make_shared<mavsdk::Param>(system_);
    telem_   = std::make_shared<mavsdk::Telemetry>(system_);

    // register some callbacks
    const mavsdk::Mission::MissionProgressCallback progress_cbk = std::bind(&ControlInterface::missionProgressCallback, this, _1);
    mission_->subscribe_mission_progress(progress_cbk);

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
// the following mutexes have to be locked by the calling function:
// pose_mutex_
// action_mutex_
// telem_mutex_
// waypoint_buffer_mutex_
bool ControlInterface::startTakeoff(std::string& fail_reason_out)
{
  std::stringstream ss;
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

  if (pose_takeoff_samples_.size() < (size_t)takeoff_position_samples_)
  {
    ss << "need " << takeoff_position_samples_ << " odometry samples, only have " << pose_takeoff_samples_.size();
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

  local_waypoint_t current_goal;

  // averaging desired takeoff position
  current_goal.x = 0;
  current_goal.y = 0;

  for (const auto &p : pose_takeoff_samples_)
  {
    current_goal.x += p.x();
    current_goal.y += p.y();
  }
  current_goal.x /= pose_takeoff_samples_.size();
  current_goal.y /= pose_takeoff_samples_.size();

  current_goal.z   = takeoff_height_;
  current_goal.yaw = getYaw(pose_ori_);
  desired_pose_    = Eigen::Vector4d(current_goal.x, current_goal.y, current_goal.z, current_goal.yaw);

  waypoint_buffer_.push_back(current_goal);

  RCLCPP_WARN(get_logger(), "Takeoff action called.");
  return true;
}
//}

/* startLanding //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// action_mutex_
bool ControlInterface::startLanding(std::string& fail_reason_out)
{
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

/* startMission //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// mission_mutex_
bool ControlInterface::startMission()
{
  if (manual_override_)
  {
    RCLCPP_WARN(get_logger(), "Mission start prevented by manual override");
    return false;
  }

  const auto result = mission_->start_mission();
  if (result != mavsdk::Mission::Result::Success)
  {
    RCLCPP_ERROR(get_logger(), "Mission start rejected with exit symbol %s", to_string(result).c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "Mission started");
  return true;
}
//}

/* startMissionUpload //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// mission_mutex_
// mission_upload_mutex_
bool ControlInterface::startMissionUpload(const mavsdk::Mission::MissionPlan& mission_plan)
{
  mission_upload_state_ = mission_upload_state_t::failed;

  if (mission_plan.mission_items.empty())
  {
    RCLCPP_ERROR(get_logger(), "Mission waypoints empty. Nothing to upload");
    return false;
  }

  if (manual_override_)
  {
    RCLCPP_WARN(get_logger(), "Mission upload prevented by manual override");
    return false;
  }

  mission_upload_start_time_ = get_clock()->now();
  mission_->upload_mission_async(mission_plan, [this](mavsdk::Mission::Result result)
      {
        // spawn a new thread to avoid blocking in the MavSDK
        // callback which will eventually cause a deadlock
        // of the MavSDK processing thread
        std::thread([this, result]
        {
          std::scoped_lock lck(mission_upload_mutex_);
          mission_upload_end_time_ = get_clock()->now();
          if (result == mavsdk::Mission::Result::Success)
            mission_upload_state_ = mission_upload_state_t::done;
          else
            mission_upload_state_ = mission_upload_state_t::failed;
        }).detach();
      }
    );
  RCLCPP_INFO(get_logger(), "Started mission upload attempt #%d", mission_upload_attempts_);

  mission_upload_state_ = mission_upload_state_t::started;
  return true;
}
//}

/* stopMission //{ */
// the following mutexes have to be locked by the calling function:
// waypoint_buffer_mutex_
// mission_mutex_
// mission_upload_mutex_
bool ControlInterface::stopMission(std::string& fail_reason_out)
{
  std::stringstream ss;
  if (mission_state_ == mission_state_t::finished)
    return true;

  // clear and reset stuff
  waypoint_buffer_.clear();
  mission_upload_waypoints_.mission_items.clear();

  // cancel any current mission upload to pixhawk if applicable
  if (mission_upload_state_ == mission_upload_state_t::started)
  {
    const auto cancel_result = mission_->cancel_mission_upload();
    if (cancel_result != mavsdk::Mission::Result::Success)
    {
      ss << "cannot stop mission upload (" << to_string(cancel_result) << ")";
      fail_reason_out = ss.str();
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to stop current mission: " << fail_reason_out);
      return false;
    }
  }

  // clear any currently uploaded mission
  const auto clear_result = mission_->clear_mission();
  if (clear_result != mavsdk::Mission::Result::Success)
  {
    ss << "cannot clear current mission (" << to_string(clear_result) << ")";
    fail_reason_out = ss.str();
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to stop current mission: " << fail_reason_out);
    return false;
  }

  mission_state_ = mission_state_t::finished;
  RCLCPP_INFO(get_logger(), "Current mission stopped");
  return true;
}
//}

// | ---------- Diagnostics and debug helper methods ---------- |

/* publishDiagnostics //{ */
// the following mutexes have to be locked by the calling function:
// state_mutex_
// mission_mutex_
// mission_progress_mutex_
// telem_mutex_
void ControlInterface::publishDiagnostics()
{
  const bool armed = telem_ != nullptr && telem_->armed();

  fog_msgs::msg::ControlInterfaceDiagnostics msg;
  msg.header.stamp         = get_clock()->now();
  msg.header.frame_id      = world_frame_;
  msg.armed                = armed;
  msg.airborne             = vehicle_state_ == vehicle_state_t::autonomous_flight || vehicle_state_ == vehicle_state_t::manual_flight;
  msg.moving               = mission_state_ == mission_state_t::in_progress;
  msg.mission_finished     = mission_state_ == mission_state_t::finished;
  msg.mission_size         = mission_progress_size_;
  msg.mission_waypoint     = mission_progress_current_waypoint_;
  msg.gps_origin_set       = gps_origin_set_;
  msg.getting_control_mode = getting_control_mode_;
  msg.getting_odom         = getting_odom_;
  msg.manual_control       = manual_override_;

  diagnostics_publisher_->publish(msg);
}
//}

/* publishDesiredPose //{ */
void ControlInterface::publishDesiredPose()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp     = get_clock()->now();
  msg.header.frame_id  = world_frame_;
  msg.pose.position.x  = desired_pose_.x();
  msg.pose.position.y  = desired_pose_.y();
  msg.pose.position.z  = desired_pose_.z();
  const Eigen::Quaterniond q(Eigen::AngleAxisd(desired_pose_.w(), Eigen::Vector3d::UnitZ()));
  msg.pose.orientation.w = q.w();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  desired_pose_publisher_->publish(msg);
}
//}

/* publishDebugMarkers //{ */
void ControlInterface::publishDebugMarkers()
{
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp    = get_clock()->now();
  msg.header.frame_id = world_frame_;

  for (auto &w : waypoint_buffer_) {
    geometry_msgs::msg::Pose p;
    p.position.x = w.x;
    p.position.y = w.y;
    p.position.z = w.z;
    const Eigen::Quaterniond q(Eigen::AngleAxisd(w.yaw, Eigen::Vector3d::UnitZ()));
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

/* parse_param //{ */
template <class T>
bool ControlInterface::parse_param(const std::string &param_name, T &param_dest)
{
#ifdef ROS_FOXY
  declare_parameter(param_name); // for Foxy
#else
  declare_parameter<T>(param_name); // for Galactic and newer
#endif
  if (!get_parameter(param_name, param_dest))
  {
    RCLCPP_ERROR(get_logger(), "Could not load param '%s'", param_name.c_str());
    return false;
  }
  else
  {
    RCLCPP_INFO_STREAM(get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
  }
  return true;
}

bool ControlInterface::parse_param(const std::string& param_name, rclcpp::Duration& param_dest)
{
  using T = double;
#ifdef ROS_FOXY
  declare_parameter(param_name); // for Foxy
#else
  declare_parameter<T>(param_name); // for Galactic and newer
#endif
  T tmp;
  if (!get_parameter(param_name, tmp))
  {
    RCLCPP_ERROR(get_logger(), "Could not load param '%s'", param_name.c_str());
    return false;
  }
  else
  {
    param_dest = rclcpp::Duration::from_seconds(tmp);
    RCLCPP_INFO_STREAM(get_logger(), "Loaded '" << param_name << "' = '" << tmp << "s'");
  }
  return true;
}
//}

/* to_mission_item //{ */
mavsdk::Mission::MissionItem ControlInterface::to_mission_item(const local_waypoint_t& w_in)
{
  local_waypoint_t w = w_in;
  // apply home offset correction
  w.x -= home_position_offset_.x();
  w.y -= home_position_offset_.y();
  w.z -= home_position_offset_.z();

  mavsdk::Mission::MissionItem item;
  gps_waypoint_t global = localToGlobal(coord_transform_, w);
  item.latitude_deg = global.latitude;
  item.longitude_deg = global.longitude;
  item.relative_altitude_m = global.altitude;
  item.yaw_deg = -radToDeg(global.yaw + yaw_offset_correction_);
  item.speed_m_s = float(target_velocity_);  // NAN = use default values. This does NOT limit vehicle max speed

  item.is_fly_through          = true;
  item.gimbal_pitch_deg        = 0.0f;
  item.gimbal_yaw_deg          = 0.0f;
  item.camera_action           = mavsdk::Mission::MissionItem::CameraAction::None;
  item.loiter_time_s           = waypoint_loiter_time_;
  item.camera_photo_interval_s = 0.0f;
  item.acceptance_radius_m     = waypoint_acceptance_radius_;

  return item;
}
//}

/* to_local_waypoint //{ */
local_waypoint_t ControlInterface::to_local_waypoint(const geometry_msgs::msg::PoseStamped& in, const bool is_global)
{
  return to_local_waypoint(Eigen::Vector4d{in.pose.position.x, in.pose.position.y, in.pose.position.z, getYaw(in.pose.orientation)}, is_global);
}

local_waypoint_t ControlInterface::to_local_waypoint(const fog_msgs::srv::WaypointToLocal::Request& in, const bool is_global)
{
  const Eigen::Vector4d as_vec {in.latitude_deg, in.longitude_deg, in.relative_altitude_m, in.yaw};
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
    gps_waypoint_t wp;
    wp.latitude  = in.x();
    wp.longitude = in.y();
    wp.altitude  = in.z();
    wp.yaw       = in.w();
    return globalToLocal(coord_transform_, wp);
  }
  else
  {
    local_waypoint_t wp;
    wp.x   = in.x();
    wp.y   = in.y();
    wp.z   = in.z();
    wp.yaw = in.w();
    return wp;
  }
}
//}

/* to_string() function //{ */
std::string to_string(const mavsdk::Action::Result result)
{
  switch (result)
  {
    case mavsdk::Action::Result::Unknown:                         return "Unknown result.";
    case mavsdk::Action::Result::Success:                         return "Request was successful.";
    case mavsdk::Action::Result::NoSystem:                        return "No system is connected.";
    case mavsdk::Action::Result::ConnectionError:                 return "Connection error.";
    case mavsdk::Action::Result::Busy:                            return "Vehicle is busy.";
    case mavsdk::Action::Result::CommandDenied:                   return "Command refused by vehicle.";
    case mavsdk::Action::Result::CommandDeniedLandedStateUnknown: return "Command refused because landed state is unknown.";
    case mavsdk::Action::Result::CommandDeniedNotLanded:          return "Command refused because vehicle not landed.";
    case mavsdk::Action::Result::Timeout:                         return "Request timed out.";
    case mavsdk::Action::Result::VtolTransitionSupportUnknown:    return "Hybrid/VTOL transition support is unknown.";
    case mavsdk::Action::Result::NoVtolTransitionSupport:         return "Vehicle does not support hybrid/VTOL transitions.";
    case mavsdk::Action::Result::ParameterError:                  return "Error getting or setting parameter.";
  }
  return "Invalid result.";
}

std::string to_string(const mavsdk::Mission::Result result)
{
  switch (result)
  {
    case mavsdk::Mission::Result::Unknown:                return "Unknown result.";
    case mavsdk::Mission::Result::Success:                return "Request succeeded.";
    case mavsdk::Mission::Result::Error:                  return "Error.";
    case mavsdk::Mission::Result::TooManyMissionItems:    return "Too many mission items in the mission.";
    case mavsdk::Mission::Result::Busy:                   return "Vehicle is busy.";
    case mavsdk::Mission::Result::Timeout:                return "Request timed out.";
    case mavsdk::Mission::Result::InvalidArgument:        return "Invalid argument.";
    case mavsdk::Mission::Result::Unsupported:            return "Mission downloaded from the system is not supported.";
    case mavsdk::Mission::Result::NoMissionAvailable:     return "No mission available on the system.";
    case mavsdk::Mission::Result::UnsupportedMissionCmd:  return "Unsupported mission command.";
    case mavsdk::Mission::Result::TransferCancelled:      return "Mission transfer (upload or download) has been cancelled.";
    case mavsdk::Mission::Result::NoSystem:               return "No system connected.";
  }
  return "Invalid result.";
}

std::string to_string(const mavsdk::Param::Result result)
{
  switch (result)
  {
    case mavsdk::Param::Result::Unknown:          return "Unknown result.";
    case mavsdk::Param::Result::Success:          return "Request succeeded.";
    case mavsdk::Param::Result::Timeout:          return "Request timed out.";
    case mavsdk::Param::Result::ConnectionError:  return "Connection error.";
    case mavsdk::Param::Result::WrongType:        return "Wrong type.";
    case mavsdk::Param::Result::ParamNameTooLong: return "Parameter name too long (> 16).";
    case mavsdk::Param::Result::NoSystem:         return "No system connected.";
  }
  return "Invalid result.";
}

std::string to_string(const mavsdk::ConnectionResult result)
{
  switch (result)
  {
    case mavsdk::ConnectionResult::Success:               return "Connection succeeded.";
    case mavsdk::ConnectionResult::Timeout:               return "Connection timed out.";
    case mavsdk::ConnectionResult::SocketError:           return "Socket error.";
    case mavsdk::ConnectionResult::BindError:             return "Bind error.";
    case mavsdk::ConnectionResult::SocketConnectionError: return "Socket connection error.";
    case mavsdk::ConnectionResult::ConnectionError:       return "Connection error.";
    case mavsdk::ConnectionResult::NotImplemented:        return "Connection type not implemented.";
    case mavsdk::ConnectionResult::SystemNotConnected:    return "No system is connected.";
    case mavsdk::ConnectionResult::SystemBusy:            return "System is busy.";
    case mavsdk::ConnectionResult::CommandDenied:         return "Command is denied.";
    case mavsdk::ConnectionResult::DestinationIpUnknown:  return "Connection IP is unknown.";
    case mavsdk::ConnectionResult::ConnectionsExhausted:  return "Connections exhausted.";
    case mavsdk::ConnectionResult::ConnectionUrlInvalid:  return "URL invalid.";
    case mavsdk::ConnectionResult::BaudrateUnknown:       return "Baudrate unknown.";
  }
  return "Invalid result.";
}

std::string to_string(const mavsdk::Telemetry::FlightMode mode)
{
  switch (mode)
  {
    case mavsdk::Telemetry::FlightMode::Unknown:        return "Mode not known.";
    case mavsdk::Telemetry::FlightMode::Ready:          return "Armed and ready to take off.";
    case mavsdk::Telemetry::FlightMode::Takeoff:        return "Taking off.";
    case mavsdk::Telemetry::FlightMode::Hold:           return "Holding (hovering in place (or circling for fixed-wing vehicles).";
    case mavsdk::Telemetry::FlightMode::Mission:        return "In mission.";
    case mavsdk::Telemetry::FlightMode::ReturnToLaunch: return "Returning to launch position (then landing).";
    case mavsdk::Telemetry::FlightMode::Land:           return "Landing.";
    case mavsdk::Telemetry::FlightMode::Offboard:       return "In 'offboard' mode.";
    case mavsdk::Telemetry::FlightMode::FollowMe:       return "In 'follow-me' mode.";
    case mavsdk::Telemetry::FlightMode::Manual:         return "In 'Manual' mode.";
    case mavsdk::Telemetry::FlightMode::Altctl:         return "In 'Altitude Control' mode.";
    case mavsdk::Telemetry::FlightMode::Posctl:         return "In 'Position Control' mode.";
    case mavsdk::Telemetry::FlightMode::Acro:           return "In 'Acro' mode.";
    case mavsdk::Telemetry::FlightMode::Stabilized:     return "In 'Stabilize' mode.";
    case mavsdk::Telemetry::FlightMode::Rattitude:      return "In 'Rattitude' mode.";
  }
  return "Invalid flight mode.";
}

std::string to_string(const mavsdk::Telemetry::LandedState land_state)
{
  switch (land_state)
  {
    case mavsdk::Telemetry::LandedState::Unknown:   return "Landed state is unknown.";
    case mavsdk::Telemetry::LandedState::OnGround:  return "The vehicle is on the ground.";
    case mavsdk::Telemetry::LandedState::InAir:     return "The vehicle is in the air.";
    case mavsdk::Telemetry::LandedState::TakingOff: return "The vehicle is taking off.";
    case mavsdk::Telemetry::LandedState::Landing:   return "The vehicle is landing.";
  }
  return "Invalid result.";
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
