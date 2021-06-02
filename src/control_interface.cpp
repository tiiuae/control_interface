#include <eigen3/Eigen/Core>
#include <fog_msgs/srv/path.hpp>
#include <fog_msgs/srv/vec4.hpp>
#include <fog_msgs/msg/control_interface_diagnostics.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mavsdk/geometry.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/mission_result.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::placeholders;

namespace control_interface
{

/* class ControlInterface //{ */
class ControlInterface : public rclcpp::Node {
public:
  ControlInterface(rclcpp::NodeOptions options);

private:
  bool is_initialized_       = false;
  bool getting_timesync_     = false;
  bool getting_gps_          = false;
  bool getting_pixhawk_odom_ = false;
  bool getting_landed_info_  = false;
  bool getting_control_mode_ = false;
  bool start_mission_        = false;
  bool mission_received_     = false;
  bool armed_                = false;
  bool takeoff_requested_    = false;
  bool motion_started_       = false;
  bool landed_               = true;

  bool     mission_finished_      = true;
  unsigned last_mission_instance_ = 1;

  std::string uav_name_         = "";
  std::string world_frame_      = "";
  std::string ned_origin_frame_ = "";
  std::string ned_fcu_frame_    = "";
  std::string fcu_frame_        = "";

  std::string                      device_url_;
  mavsdk::Mavsdk                   mavsdk_;
  std::shared_ptr<mavsdk::System>  system_;
  std::shared_ptr<mavsdk::Action>  action_;
  std::shared_ptr<mavsdk::Mission> mission_;
  mavsdk::Mission::MissionPlan     mission_plan_;

  std::vector<Eigen::Vector3d> waypoint_buffer_;

  std::shared_ptr<tf2_ros::Buffer>                     tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener>          tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  // vehicle global position
  float latitude_, longitude_, altitude_;

  // vehicle local position
  Eigen::Vector3d pos_;
  float           ori_[4];

  // use takeoff lat and long to initialize local frame
  std::shared_ptr<mavsdk::geometry::CoordinateTransformation> coord_transform_;

  // config params
  double takeoff_height_        = 2.5;
  double waypoint_marker_scale_ = 0.3;
  double control_loop_rate_     = 20.0;
  double waypoint_loiter_time_  = 0.0;

  std::atomic<unsigned long long> timestamp_;

  // publishers
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr              vehicle_command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr                    local_odom_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr       marker_publisher_;
  rclcpp::Publisher<fog_msgs::msg::ControlInterfaceDiagnostics>::SharedPtr diagnostics_publisher_;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr              timesync_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr       pixhawk_odom_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr    control_mode_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr   land_detected_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::MissionResult>::SharedPtr         mission_result_subscriber_;

  // subscriber callbacks
  void timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg);
  void gpsCallback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg);
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void controlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);
  void landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg);
  void missionResultCallback(const px4_msgs::msg::MissionResult::UniquePtr msg);

  // services provided
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arming_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr takeoff_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr land_service_;
  rclcpp::Service<fog_msgs::srv::Vec4>::SharedPtr    local_setpoint_service_;
  rclcpp::Service<fog_msgs::srv::Path>::SharedPtr    set_waypoints_service_;

  // service callbacks
  bool armingCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool takeoffCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool landCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  bool localSetpointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request, std::shared_ptr<fog_msgs::srv::Vec4::Response> response);
  bool setWaypointsCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request, std::shared_ptr<fog_msgs::srv::Path::Response> response);

  // internal functions
  bool gettingPixhawkSensors();
  void printSensorsStatus();
  void publishDiagnostics();

  bool takeoff();
  bool land();
  bool startMission();
  bool uploadMission();
  bool stopPreviousMission();

  void addGlobalToMission(Eigen::Vector3d waypoint);
  void addLocalToMission(Eigen::Vector3d waypoint);
  void publishTF();
  void publishStaticTF();
  void publishLocalOdom();
  void publishDebugMarkers();

  geometry_msgs::msg::PoseStamped transformBetween(std::string frame_from, std::string frame_to);
  std_msgs::msg::ColorRGBA        generateColor(const double r, const double g, const double b, const double a);

  // timers
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr     control_timer_;
  void                             controlRoutine(void);

  // utils
  template <class T>
  bool parse_param(std::string param_name, T &param_dest);
};
//}

/* constructor //{ */
ControlInterface::ControlInterface(rclcpp::NodeOptions options) : Node("control_interface", options) {

  RCLCPP_INFO(this->get_logger(), "Initializing...");

  try {
    uav_name_ = std::string(std::getenv("DRONE_DEVICE_ID"));
  }
  catch (...) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Environment variable DRONE_DEVICE_ID was not defined!", this->get_name());
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: UAV name is: '%s'", this->get_name(), uav_name_.c_str());

  /* parse params from config file //{ */
  parse_param("device_url", device_url_);
  parse_param("takeoff_height", takeoff_height_);
  parse_param("waypoint_marker_scale", waypoint_marker_scale_);
  parse_param("waypoint_loiter_time", waypoint_loiter_time_);

  /* frame definition */
  world_frame_      = "world";
  fcu_frame_        = uav_name_ + "/fcu";
  ned_fcu_frame_    = uav_name_ + "/ned_fcu";
  ned_origin_frame_ = uav_name_ + "/ned_origin";
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
      RCLCPP_INFO(this->get_logger(), "[%s]: ID: %u", this->get_name(), i);
      if (mavsdk_.systems().at(i)->get_system_id() == 1) {
        connected = true;
        system_   = mavsdk_.systems().at(i);
        break;
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: Target connected", this->get_name());
  action_  = std::make_shared<mavsdk::Action>(system_);
  mission_ = std::make_shared<mavsdk::Mission>(system_);
  //}

  // publishers
  vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("~/vehicle_command_out", 10);
  local_odom_publisher_      = this->create_publisher<nav_msgs::msg::Odometry>("~/local_odom_out", 10);
  marker_publisher_          = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug_markers_out", 10);
  diagnostics_publisher_     = this->create_publisher<fog_msgs::msg::ControlInterfaceDiagnostics>("~/diagnostics_out", 10);

  // subscribers
  timesync_subscriber_ = this->create_subscription<px4_msgs::msg::Timesync>("~/timesync_in", 10, std::bind(&ControlInterface::timesyncCallback, this, _1));
  gps_subscriber_      = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>("~/gps_in", 10, std::bind(&ControlInterface::gpsCallback, this, _1));
  pixhawk_odom_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleOdometry>("~/pixhawk_odom_in", 10, std::bind(&ControlInterface::pixhawkOdomCallback, this, _1));
  control_mode_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleControlMode>("~/control_mode_in", 10, std::bind(&ControlInterface::controlModeCallback, this, _1));
  land_detected_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleLandDetected>("~/land_detected_in", 10, std::bind(&ControlInterface::landDetectedCallback, this, _1));
  mission_result_subscriber_ =
      this->create_subscription<px4_msgs::msg::MissionResult>("~/mission_result_in", 10, std::bind(&ControlInterface::missionResultCallback, this, _1));

  // service handlers
  arming_service_         = this->create_service<std_srvs::srv::SetBool>("~/arming_in", std::bind(&ControlInterface::armingCallback, this, _1, _2));
  takeoff_service_        = this->create_service<std_srvs::srv::SetBool>("~/takeoff_in", std::bind(&ControlInterface::takeoffCallback, this, _1, _2));
  land_service_           = this->create_service<std_srvs::srv::SetBool>("~/land_in", std::bind(&ControlInterface::landCallback, this, _1, _2));
  local_setpoint_service_ = this->create_service<fog_msgs::srv::Vec4>("~/local_setpoint_in", std::bind(&ControlInterface::localSetpointCallback, this, _1, _2));
  set_waypoints_service_  = this->create_service<fog_msgs::srv::Path>("~/waypoints_in", std::bind(&ControlInterface::setWaypointsCallback, this, _1, _2));

  control_timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0 / control_loop_rate_), std::bind(&ControlInterface::controlRoutine, this), callback_group_);

  tf_broadcaster_        = nullptr;
  static_tf_broadcaster_ = nullptr;

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: Initialized", this->get_name());
}
//}

/* timesyncCallback //{ */
void ControlInterface::timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }
  getting_timesync_ = true;
  timestamp_.store(msg->timestamp);
}
//}

/* gpsCallback //{ */
void ControlInterface::gpsCallback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  if (!getting_gps_) {
    mavsdk::geometry::CoordinateTransformation::GlobalCoordinate ref;
    ref.latitude_deg  = msg->lat;
    ref.longitude_deg = msg->lon;
    coord_transform_  = std::make_shared<mavsdk::geometry::CoordinateTransformation>(mavsdk::geometry::CoordinateTransformation(ref));
  }

  this->latitude_  = msg->lat;
  this->longitude_ = msg->lon;
  this->altitude_  = msg->alt;
  getting_gps_     = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting gps!", this->get_name());
}
//}

/* pixhawkOdomCallback //{ */
void ControlInterface::pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  pos_.x() = msg->x;
  pos_.y() = msg->y;
  pos_.z() = msg->z;
  ori_[0]  = msg->q[0];
  ori_[1]  = msg->q[1];
  ori_[2]  = msg->q[2];
  ori_[3]  = msg->q[3];

  publishTF();
  publishLocalOdom();

  getting_pixhawk_odom_ = true;
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting pixhawk odometry!", this->get_name());

  // one-shot publish static TF
  if (static_tf_broadcaster_ == nullptr) {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this->shared_from_this());
    publishStaticTF();
  }
}
//}

/* controlModeCallback //{ */
void ControlInterface::controlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  getting_control_mode_ = true;

  if (armed_ != msg->flag_armed) {
    armed_ = msg->flag_armed;
    if (armed_) {
      RCLCPP_WARN(this->get_logger(), "[%s]: Vehicle armed", this->get_name());
    } else {
      takeoff_requested_ = false;
      start_mission_     = false;
      motion_started_    = false;
      RCLCPP_WARN(this->get_logger(), "[%s]: Vehicle disarmed", this->get_name());
    }
  }
}
//}

/* landDetectedCallback //{ */
void ControlInterface::landDetectedCallback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }
  getting_landed_info_ = true;
  landed_              = msg->landed;
}
//}

/* missionResultCallback //{ */
void ControlInterface::missionResultCallback(const px4_msgs::msg::MissionResult::UniquePtr msg) {
  if (!is_initialized_) {
    return;
  }

  unsigned instance = msg->instance_count;

  if (msg->finished && instance != last_mission_instance_) {
    mission_finished_      = true;
    last_mission_instance_ = msg->instance_count;
  }
}
//}

/* takeoffCallback //{ */
bool ControlInterface::takeoffCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                       std::shared_ptr<std_srvs::srv::SetBool::Response>                       response) {

  if (!is_initialized_) {
    response->success = false;
    response->message = "Takeoff rejected, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gettingPixhawkSensors()) {
    response->success = false;
    response->message = "Takeoff rejected, missing Pixhawk sensors";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!armed_) {
    response->success = false;
    response->message = "Takeoff rejected, vehicle not armed";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!landed_) {
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
bool ControlInterface::landCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response>                       response) {

  if (!is_initialized_) {
    response->success = false;
    response->message = "Landing rejected, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gettingPixhawkSensors()) {
    response->success = false;
    response->message = "Landing rejected, missing Pixhawk sensors";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!armed_) {
    response->success = false;
    response->message = "Landing rejected, vehicle not armed";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (landed_) {
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

  if (!is_initialized_) {
    response->success = false;
    response->message = "Arming rejected, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gettingPixhawkSensors()) {
    response->success = false;
    response->message = "Arming rejected, missing Pixhawk sensors";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

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
      armed_            = true;
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
      armed_            = false;
      RCLCPP_WARN(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
      return true;
    }
  }
}
//}

/* localSetpointCallback //{ */
bool ControlInterface::localSetpointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                             std::shared_ptr<fog_msgs::srv::Vec4::Response>      response) {

  if (!is_initialized_) {
    response->success = false;
    response->message = "Setpoint not set, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gettingPixhawkSensors()) {
    response->success = false;
    response->message = "Setpoint not set, missing Pixhawk sensors";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (landed_) {
    response->success = false;
    response->message = "Setpoint not set, vehicle not airborne";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!stopPreviousMission()) {
    response->success = false;
    response->message = "Setpoint not set, previous mission cannot be aborted";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  response->message = "Setpoint set";
  response->success = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());

  Eigen::Vector3d goal(request->goal[0], request->goal[1], request->goal[2]);
  waypoint_buffer_.push_back(goal);
  motion_started_ = true;
  return true;
}
//}

/* setWaypointsCallback //{ */
bool ControlInterface::setWaypointsCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request,
                                            std::shared_ptr<fog_msgs::srv::Path::Response>      response) {

  if (!is_initialized_) {
    response->success = false;
    response->message = "Waypoints not set, not initialized";
    RCLCPP_ERROR(this->get_logger(), "[%s]: %s", this->get_name(), response->message.c_str());
    return true;
  }

  if (!gettingPixhawkSensors()) {
    response->success = false;
    response->message = "Waypoints not set, missing Pixhawk sensors";
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

  RCLCPP_INFO(this->get_logger(), "[%s]: Got %d waypoints", this->get_name(), request->path.poses.size());
  for (size_t i = 0; i < request->path.poses.size(); i++) {
    Eigen::Vector3d wp;
    wp.x() = request->path.poses[i].pose.position.x;
    wp.y() = request->path.poses[i].pose.position.y;
    wp.z() = request->path.poses[i].pose.position.z;
    waypoint_buffer_.push_back(wp);
  }
  motion_started_   = true;
  response->success = true;
  response->message = "Waypoints set";
  return true;
}
//}

/* controlRoutine //{ */
void ControlInterface::controlRoutine(void) {

  if (is_initialized_) {
    publishDiagnostics();

    if (gettingPixhawkSensors()) {

      RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: CONTROL INTERFACE IS READY", this->get_name());

      if (!armed_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Vehicle not armed", this->get_name());
        return;
      }

      if (landed_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: Vehicle not airborne", this->get_name());
        return;
      }

      /* handle motion //{ */
      if (motion_started_) {

        // create a new mission plan if there are unused points in buffer
        if (waypoint_buffer_.size() > 0 && mission_finished_) {
          publishDebugMarkers();
          RCLCPP_INFO(this->get_logger(), "[%s]: Waypoints to be visited: %ld", this->get_name(), waypoint_buffer_.size());
          mission_->pause_mission();
          mission_plan_.mission_items.clear();
          addLocalToMission(*waypoint_buffer_.begin());
          waypoint_buffer_.erase(waypoint_buffer_.begin());
          start_mission_ = true;
        }

        // upload and execute new mission
        if (start_mission_ && mission_plan_.mission_items.size() > 0) {
          uploadMission();
          startMission();
          mission_finished_ = false;
          start_mission_    = false;
        }

        // stop if final goal is reached
        if (mission_finished_) {
          RCLCPP_INFO(this->get_logger(), "[%s]: All waypoints have been visited", this->get_name());
          motion_started_ = false;
        }
      }
      //}

    } else {
      printSensorsStatus();
    }
  }
}
//}

/* gettingPixhawkSensors //{ */
bool ControlInterface::gettingPixhawkSensors() {
  return getting_timesync_ && getting_gps_ && getting_pixhawk_odom_ && getting_control_mode_ && getting_landed_info_;
}
//}

/* printSensorsStatus //{ */
void ControlInterface::printSensorsStatus() {
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[%s]: TIME:%s, GPS:%s, ODOM:%s, CTRL:%s, LAND:%s", this->get_name(),
                       getting_timesync_ ? "TRUE" : "FALSE", getting_gps_ ? "TRUE" : "FALSE", getting_pixhawk_odom_ ? "TRUE" : "FALSE",
                       getting_control_mode_ ? "TRUE" : "FALSE", getting_landed_info_ ? "TRUE" : "FALSE");
}
//}

/* publishDiagnostics //{ */
void ControlInterface::publishDiagnostics() {
  fog_msgs::msg::ControlInterfaceDiagnostics msg;
  msg.armed                = armed_;
  msg.airborne             = !landed_;
  msg.moving               = motion_started_;
  msg.mission_finished     = mission_finished_;
  msg.waypoints_to_go      = waypoint_buffer_.size();
  msg.getting_timesync     = getting_timesync_;
  msg.getting_gps          = getting_gps_;
  msg.getting_odom         = getting_pixhawk_odom_;
  msg.getting_control_mode = getting_control_mode_;
  msg.getting_land_sensor  = getting_landed_info_;
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
  action_->takeoff();
  if (result != mavsdk::Action::Result::Success) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Takeoff failed", this->get_name());
    return false;
  }
  Eigen::Vector3d current_goal(pos_.x(), pos_.y(), takeoff_height_);
  waypoint_buffer_.push_back(current_goal);
  motion_started_ = true;
  RCLCPP_INFO(this->get_logger(), "[%s]: Taking off", this->get_name());
  return true;
}
//}

/* land //{ */
bool ControlInterface::land() {
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

  if (!motion_started_) {
    return true;
  }

  motion_started_   = false;
  start_mission_    = false;
  mission_finished_ = true;

  auto result = mission_->pause_mission();
  mission_plan_.mission_items.clear();
  waypoint_buffer_.clear();

  if (result != mavsdk::Mission::Result::Success || mission_plan_.mission_items.size() > 0) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Previous mission cannot be stopped", this->get_name());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Previous mission stopped", this->get_name());
  return true;
}
//}

/* addGlobalToMission //{ */
void ControlInterface::addGlobalToMission(Eigen::Vector3d waypoint) {
  mavsdk::Mission::MissionItem item;
  item.latitude_deg            = waypoint[0];
  item.longitude_deg           = waypoint[1];
  item.relative_altitude_m     = waypoint[2];
  item.speed_m_s               = NAN;  // NAN = use default values. This does NOT limit vehicle max speed
  item.is_fly_through          = false;
  item.gimbal_pitch_deg        = 0.0f;
  item.gimbal_yaw_deg          = 0.0f;
  item.camera_action           = mavsdk::Mission::MissionItem::CameraAction::None;
  item.loiter_time_s           = waypoint_loiter_time_;
  item.camera_photo_interval_s = 0.0f;
  mission_plan_.mission_items.push_back(item);
  RCLCPP_INFO(this->get_logger(), "[%s]: Waypoint (GPS) [%.2f,%.2f,%.2f] added into mission", this->get_name(), item.latitude_deg, item.longitude_deg,
              item.relative_altitude_m);
}
//}

/* addLocalToMission //{ */
void ControlInterface::addLocalToMission(Eigen::Vector3d waypoint) {
  mavsdk::Mission::MissionItem                                item;
  mavsdk::geometry::CoordinateTransformation::LocalCoordinate local;
  local.north_m = waypoint[1];
  local.east_m  = waypoint[0];

  auto global                  = coord_transform_->global_from_local(local);
  item.latitude_deg            = global.latitude_deg;
  item.longitude_deg           = global.longitude_deg;
  item.relative_altitude_m     = waypoint[2];
  item.speed_m_s               = NAN;  // NAN = use default values. This does NOT limit vehicle max speed
  item.is_fly_through          = false;
  item.gimbal_pitch_deg        = 0.0f;
  item.gimbal_yaw_deg          = 0.0f;
  item.camera_action           = mavsdk::Mission::MissionItem::CameraAction::None;
  item.loiter_time_s           = waypoint_loiter_time_;
  item.camera_photo_interval_s = 0.0f;
  mission_plan_.mission_items.push_back(item);
  RCLCPP_INFO(this->get_logger(), "[%s]: Waypoint (LOCAL) [%.2f,%.2f,%.2f] added into mission", this->get_name(), waypoint[0], waypoint[1], waypoint[2]);
}
//}

/* publishStaticTF //{ */
void ControlInterface::publishStaticTF() {

  geometry_msgs::msg::TransformStamped tf_stamped;
  tf2::Quaternion                      q;
  q.setRPY(-M_PI, 0, 0);
  tf_stamped.header.frame_id         = ned_fcu_frame_;
  tf_stamped.child_frame_id          = fcu_frame_;
  tf_stamped.transform.translation.x = 0.0;
  tf_stamped.transform.translation.y = 0.0;
  tf_stamped.transform.translation.z = 0.0;
  tf_stamped.transform.rotation.x    = q.getX();
  tf_stamped.transform.rotation.y    = q.getY();
  tf_stamped.transform.rotation.z    = q.getZ();
  tf_stamped.transform.rotation.w    = q.getW();
  static_tf_broadcaster_->sendTransform(tf_stamped);

  q.setRPY(M_PI, 0, M_PI / 2);
  q                                  = q.inverse();
  tf_stamped.header.frame_id         = world_frame_;
  tf_stamped.child_frame_id          = ned_origin_frame_;
  tf_stamped.transform.translation.x = 0.0;
  tf_stamped.transform.translation.y = 0.0;
  tf_stamped.transform.translation.z = 0.0;
  tf_stamped.transform.rotation.x    = q.getX();
  tf_stamped.transform.rotation.y    = q.getY();
  tf_stamped.transform.rotation.z    = q.getZ();
  tf_stamped.transform.rotation.w    = q.getW();
  static_tf_broadcaster_->sendTransform(tf_stamped);
}
//}

/* publishTF //{ */
void ControlInterface::publishTF() {
  if (tf_broadcaster_ == nullptr) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }
  geometry_msgs::msg::TransformStamped tf1;
  tf1.header.stamp            = this->get_clock()->now();
  tf1.header.frame_id         = ned_origin_frame_;
  tf1.child_frame_id          = ned_fcu_frame_;
  tf1.transform.translation.x = pos_.x();
  tf1.transform.translation.y = pos_.y();
  tf1.transform.translation.z = pos_.z();
  tf1.transform.rotation.w    = ori_[0];
  tf1.transform.rotation.x    = ori_[1];
  tf1.transform.rotation.y    = ori_[2];
  tf1.transform.rotation.z    = ori_[3];
  tf_broadcaster_->sendTransform(tf1);
}
//}

/* publishLocalOdom //{ */
void ControlInterface::publishLocalOdom() {
  nav_msgs::msg::Odometry msg;
  msg.header.stamp            = this->get_clock()->now();
  msg.header.frame_id         = world_frame_;
  msg.child_frame_id          = fcu_frame_;
  auto tf                     = transformBetween(fcu_frame_, world_frame_);
  msg.pose.pose.position.x    = tf.pose.position.x;
  msg.pose.pose.position.y    = tf.pose.position.y;
  msg.pose.pose.position.z    = tf.pose.position.z;
  msg.pose.pose.orientation.w = tf.pose.orientation.w;
  msg.pose.pose.orientation.x = tf.pose.orientation.x;
  msg.pose.pose.orientation.y = tf.pose.orientation.y;
  msg.pose.pose.orientation.z = tf.pose.orientation.z;
  local_odom_publisher_->publish(msg);
}
//}

/* publishDebugMarkers //{ */
void ControlInterface::publishDebugMarkers() {
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker points_marker;
  points_marker.header.stamp       = this->get_clock()->now();
  points_marker.header.frame_id    = world_frame_;
  points_marker.ns                 = uav_name_ + "/debug/waypoints";
  points_marker.type               = visualization_msgs::msg::Marker::POINTS;
  points_marker.id                 = 8;
  points_marker.action             = visualization_msgs::msg::Marker::ADD;
  points_marker.pose.orientation.w = 1.0;
  points_marker.scale.x            = waypoint_marker_scale_;
  points_marker.scale.y            = waypoint_marker_scale_;

  for (auto &wp : waypoint_buffer_) {
    std_msgs::msg::ColorRGBA  color = generateColor(0, 1, 0, 1);
    geometry_msgs::msg::Point gp;
    gp.x = wp.x();
    gp.y = wp.y();
    gp.z = wp.z();
    points_marker.points.push_back(gp);
    points_marker.colors.push_back(color);
  }
  msg.markers.push_back(points_marker);
  marker_publisher_->publish(msg);
}
//}

/* transformBetween //{ */
geometry_msgs::msg::PoseStamped ControlInterface::transformBetween(std::string frame_from, std::string frame_to) {
  geometry_msgs::msg::PoseStamped pose_out;
  try {
    auto transform_stamped      = tf_buffer_->lookupTransform(frame_to, frame_from, rclcpp::Time(0));
    pose_out.pose.position.x    = transform_stamped.transform.translation.x;
    pose_out.pose.position.y    = transform_stamped.transform.translation.y;
    pose_out.pose.position.z    = transform_stamped.transform.translation.z;
    pose_out.pose.orientation.w = transform_stamped.transform.rotation.w;
    pose_out.pose.orientation.x = transform_stamped.transform.rotation.x;
    pose_out.pose.orientation.y = transform_stamped.transform.rotation.y;
    pose_out.pose.orientation.z = transform_stamped.transform.rotation.z;
  }
  catch (...) {
  }
  return pose_out;
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
bool ControlInterface::parse_param(std::string param_name, T &param_dest) {
  const std::string param_path = "param_namespace." + param_name;
  this->declare_parameter(param_path);
  if (!this->get_parameter(param_path, param_dest)) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Could not load param '%s'", this->get_name(), param_name.c_str());
    return false;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << this->get_name() << "]: Loaded '" << param_name << "' = '" << param_dest << "'");
  }
  return true;
}
//}

/* parse_param impl //{ */
template bool ControlInterface::parse_param<int>(std::string param_name, int &param_dest);
template bool ControlInterface::parse_param<double>(std::string param_name, double &param_dest);
template bool ControlInterface::parse_param<float>(std::string param_name, float &param_dest);
template bool ControlInterface::parse_param<std::string>(std::string param_name, std::string &param_dest);
template bool ControlInterface::parse_param<unsigned int>(std::string param_name, unsigned int &param_dest);
//}

}  // namespace control_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(control_interface::ControlInterface)
