#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <thread>

#include <fog_msgs/srv/path.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <fog_msgs/srv/vec4.hpp>

#include <mavsdk/geometry.h>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>

#include <eigen3/Eigen/Core>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // This has to be here otherwise you will get cryptic linker error about missing function 'getTimestamp'
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <std_msgs/msg/color_rgba.hpp>
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
  bool getting_gps_          = false;
  bool getting_pixhawk_odom_ = false;
  bool start_mission_        = false;
  bool mission_received_     = false;
  bool armed_                = false;
  bool airborne_             = false;
  bool takeoff_requested_    = false;
  bool motion_started_       = false;

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
  Eigen::Vector3d              current_goal_;

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

  std::atomic<unsigned long long> timestamp_;

  // publishers
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr        vehicle_command_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              local_odom_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

  // subscribers
  rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr              timesync_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr gps_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr       pixhawk_odom_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr    control_mode_subscriber_;

  // subscriber callbacks
  void timesyncCallback(const px4_msgs::msg::Timesync::UniquePtr msg);
  void gpsCallback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg);
  void pixhawkOdomCallback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void controlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);

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
  bool takeoff();
  bool land();
  bool startMission();
  bool uploadMission();
  bool abortMission();

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
    RCLCPP_WARN(this->get_logger(), "[%s]: Environment variable DRONE_DEVICE_ID was not defined!", this->get_name());
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: UAV name is: '%s'", this->get_name(), uav_name_.c_str());

  /* parse params from config file //{ */
  parse_param("device_url", device_url_);
  parse_param("takeoff_height", takeoff_height_);
  parse_param("waypoint_marker_scale", waypoint_marker_scale_);

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

  // subscribers
  timesync_subscriber_ = this->create_subscription<px4_msgs::msg::Timesync>("~/timesync_in", 10, std::bind(&ControlInterface::timesyncCallback, this, _1));
  gps_subscriber_      = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>("~/gps_in", 10, std::bind(&ControlInterface::gpsCallback, this, _1));
  pixhawk_odom_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleOdometry>("~/pixhawk_odom_in", 10, std::bind(&ControlInterface::pixhawkOdomCallback, this, _1));
  control_mode_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleControlMode>("~/control_mode_in", 10, std::bind(&ControlInterface::controlModeCallback, this, _1));

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

/* setWaypointsCallback //{ */
// used internally, called by the navigation node
bool ControlInterface::setWaypointsCallback(const std::shared_ptr<fog_msgs::srv::Path::Request> request,
                                            std::shared_ptr<fog_msgs::srv::Path::Response>      response) {

  if (!abortMission()) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Setpoint not set, previous mission cannot be aborted", this->get_name());
    response->success = false;
    response->message = "Setpoint not set, previous mission cannot be aborted";
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: Got %d waypoints", this->get_name(), request->path.poses.size());
  for (size_t i = 0; i < request->path.poses.size(); i++) {
    Eigen::Vector3d wp;
    wp.x() = request->path.poses[i].pose.position.x;
    wp.y() = request->path.poses[i].pose.position.y;
    wp.z() = request->path.poses[i].pose.position.z;
    waypoint_buffer_.push_back(wp);
  }
  motion_started_ = true;
  current_goal_   = *waypoint_buffer_.begin();
  addLocalToMission(current_goal_);
  start_mission_    = true;
  response->success = true;
  response->message = "Waypoints set";
  return true;
}
//}

/* takeoffCallback //{ */
bool ControlInterface::takeoffCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                       std::shared_ptr<std_srvs::srv::SetBool::Response>                       response) {
  if (is_initialized_ && armed_ && getting_pixhawk_odom_ && getting_gps_ && !airborne_) {
    response->message  = "Takeoff started";
    response->success  = true;
    takeoff_requested_ = true;
    return true;
  }
  response->message = "Takeoff rejected";
  response->success = false;
  return false;
}
//}

/* landCallback //{ */
bool ControlInterface::landCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                    std::shared_ptr<std_srvs::srv::SetBool::Response>                       response) {
  if (is_initialized_ && getting_pixhawk_odom_ && getting_gps_ && airborne_) {
    bool success = abortMission() && land();
    if (success) {
      response->message = "Landing";
      response->success = true;
      return true;
    }
  }
  response->message = "Landing rejected";
  response->success = false;
  return false;
}
//}

/* armingCallback //{ */
bool ControlInterface::armingCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response>                       response) {

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

/* controlModeCallback //{ */
void ControlInterface::controlModeCallback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg) {
  if (armed_ != msg->flag_armed) {
    armed_ = msg->flag_armed;
    if (armed_) {
      RCLCPP_WARN(this->get_logger(), "[%s]: Vehicle armed", this->get_name());
    } else {
      RCLCPP_WARN(this->get_logger(), "[%s]: Vehicle disarmed", this->get_name());
      airborne_ = false;
    }
  }
}
//}

/* localSetpointCallback //{ */
bool ControlInterface::localSetpointCallback(const std::shared_ptr<fog_msgs::srv::Vec4::Request> request,
                                             std::shared_ptr<fog_msgs::srv::Vec4::Response>      response) {
  if (!abortMission()) {
    response->message = "Setpoint not set, previous mission cannot be aborted";
    response->success = false;
    return false;
  }

  response->message = "Setpoint set";
  response->success = true;

  Eigen::Vector3d goal(request->goal[0], request->goal[1], request->goal[2]);
  waypoint_buffer_.push_back(goal);
  motion_started_ = true;
  current_goal_   = *waypoint_buffer_.begin();
  addLocalToMission(current_goal_);
  start_mission_ = true;
  return true;
}
//}

/* controlRoutine //{ */
void ControlInterface::controlRoutine(void) {

  if (is_initialized_ && getting_gps_ && getting_pixhawk_odom_) {

    /* handle takeoff //{ */
    if (takeoff_requested_) {

      if (!armed_) {
        RCLCPP_INFO(this->get_logger(), "[%s]: Vehicle not armed", this->get_name());
        return;
      }

      if (!airborne_) {
        while (!takeoff()) {
          RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Attempting to takeoff", this->get_name());
          rclcpp::Rate(100).sleep();
        }
      }
      takeoff_requested_ = false;
    }
    //}

    /* handle motion //{ */
    if (motion_started_) {

      if (waypoint_buffer_.size() < 1) {
        RCLCPP_INFO(this->get_logger(), "[%s]: All waypoints have been visited", this->get_name());
        motion_started_ = false;
      } else {
        auto            pos_in_world_tf = tf_buffer_->lookupTransform(world_frame_, fcu_frame_, rclcpp::Time(0));
        Eigen::Vector3d pos_in_world(pos_in_world_tf.transform.translation.x, pos_in_world_tf.transform.translation.y, pos_in_world_tf.transform.translation.z);

        if ((pos_in_world - current_goal_).norm() < 0.4) {
          // current goal reached
          RCLCPP_INFO(this->get_logger(), "[%s]: Moving to the next waypoint", this->get_name());
          mission_->clear_mission();
          mission_plan_.mission_items.clear();
          current_goal_ = *waypoint_buffer_.begin();
          waypoint_buffer_.erase(waypoint_buffer_.begin());
          addLocalToMission(current_goal_);
          start_mission_ = true;
        }
      }

      if (start_mission_ && mission_plan_.mission_items.size() > 0) {
        uploadMission();

        while (!startMission()) {
          RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Attempting to start mission", this->get_name());
          rclcpp::Rate(100).sleep();
        }
        start_mission_ = false;
      }
    }
    //}

    publishDebugMarkers();
  } else {
    RCLCPP_INFO(this->get_logger(), "[%s]: Initialized: %s, GPS: %s, PX4 Odom: %s", this->get_name(), is_initialized_ ? "TRUE" : "FALSE",
                getting_gps_ ? "TRUE" : "FALSE", getting_pixhawk_odom_ ? "TRUE" : "FALSE");
  }
}
//}

/* takeoff //{ */
bool ControlInterface::takeoff() {
  auto result = action_->set_takeoff_altitude(takeoff_height_);
  if (result != mavsdk::Action::Result::Success) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Failed to set takeoff height %.2f", this->get_name(), takeoff_height_);
    return false;
  }
  action_->takeoff();
  if (result != mavsdk::Action::Result::Success) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Takeoff failed", this->get_name());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Taking off", this->get_name());
  airborne_ = true;
  return true;
}
//}

/* land //{ */
bool ControlInterface::land() {
  auto result = action_->land();
  if (result != mavsdk::Action::Result::Success) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Landing failed", this->get_name());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Landing", this->get_name());
  airborne_ = false;
  return true;
}
//}

/* startMission //{ */
bool ControlInterface::startMission() {
  auto result = mission_->start_mission();
  if (result != mavsdk::Mission::Result::Success) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Mission start rejected", this->get_name());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Mission started", this->get_name());
  return true;
}
//}

/* uploadMission //{ */
bool ControlInterface::uploadMission() {
  auto prom          = std::make_shared<std::promise<mavsdk::Mission::Result>>();
  auto future_result = prom->get_future();

  auto result = mission_->upload_mission(mission_plan_);
  if (result != mavsdk::Mission::Result::Success) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Mission upload failed", this->get_name());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Mission uploaded", this->get_name());
  return true;
}
//}

/* abortMission //{ */
bool ControlInterface::abortMission() {
  auto prom          = std::make_shared<std::promise<mavsdk::Mission::Result>>();
  auto future_result = prom->get_future();

  mission_plan_.mission_items.clear();
  waypoint_buffer_.clear();
  motion_started_ = false;
  current_goal_   = pos_;
  start_mission_  = false;
  auto result     = mission_->clear_mission();
  if (result != mavsdk::Mission::Result::Success || mission_plan_.mission_items.size() > 0) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Mission cannot be aborted", this->get_name());
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Mission aborted", this->get_name());
  return true;
}
//}

/* addGlobalToMission //{ */
void ControlInterface::addGlobalToMission(Eigen::Vector3d waypoint) {
  mavsdk::Mission::MissionItem item;
  item.latitude_deg            = waypoint[0];
  item.longitude_deg           = waypoint[1];
  item.relative_altitude_m     = waypoint[2];
  item.speed_m_s               = NAN;  // Use the default
  item.is_fly_through          = true;
  item.gimbal_pitch_deg        = 0.0f;
  item.gimbal_yaw_deg          = 0.0f;
  item.camera_action           = mavsdk::Mission::MissionItem::CameraAction::None;
  item.loiter_time_s           = 0.0f;
  item.camera_photo_interval_s = 0.0f;
  mission_plan_.mission_items.push_back(item);
  RCLCPP_INFO(this->get_logger(), "[%s]: Waypoint (GPS) [%.2f,%.2f,%.2f,%.2f] added into mission", this->get_name(), item.latitude_deg, item.longitude_deg,
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
  item.speed_m_s               = NAN;  // Use the default
  item.is_fly_through          = true;
  item.gimbal_pitch_deg        = 0.0f;
  item.gimbal_yaw_deg          = 0.0f;
  item.camera_action           = mavsdk::Mission::MissionItem::CameraAction::None;
  item.loiter_time_s           = 0.0f;
  item.camera_photo_interval_s = 0.0f;
  mission_plan_.mission_items.push_back(item);
  RCLCPP_INFO(this->get_logger(), "[%s]: Waypoint (GPS) [%.2f,%.2f,%.2f,%.2f] added into mission", this->get_name(), item.latitude_deg, item.longitude_deg,
              item.relative_altitude_m);
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
    std_msgs::msg::ColorRGBA color;
    if (wp == current_goal_) {
      color = generateColor(0, 1, 0, 1);
    } else {
      color = generateColor(0, 0.6, 0, 1);
    }
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
