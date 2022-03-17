#include "control_interface/mission_manager.h"

#include <fog_lib/mutex_utils.h>

#include <memory>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

using namespace control_interface;

// | --------------- public methods definitions --------------- |

/* constructor //{ */
MissionManager::MissionManager(const unsigned max_upload_attempts, const rclcpp::Duration& starting_timeout, std::shared_ptr<mavsdk::System> system, const rclcpp::Logger& logger, rclcpp::Clock::SharedPtr clock, rclcpp::Publisher<fog_msgs::msg::MissionPlan>::SharedPtr plan_pub, std::recursive_mutex& mutex)
  : mutex_(mutex), mission_(std::make_unique<mavsdk::Mission>(system)), max_upload_attempts_(max_upload_attempts), starting_timeout_(starting_timeout), logger_(logger), clock_(clock), plan_pub_(plan_pub)
{
    // register some callbacks
    const mavsdk::Mission::MissionProgressCallback progress_cbk = std::bind(&MissionManager::progress_callback, this, std::placeholders::_1);
    mission_->subscribe_mission_progress(progress_cbk);
}
//}

/* new_mission() method //{ */
bool MissionManager::new_mission(const mavsdk::Mission::MissionPlan& mission_plan, const uint32_t id, std::string& fail_reason_out)
{
  std::scoped_lock lck(mutex_);
  std::stringstream ss;

  if (mission_active(state_))
  {
    ss << "Last mission #" << mission_id_ << " still active (mission " << to_string(state_) << "), cannot upload new mission.";
    fail_reason_out = ss.str();
    RCLCPP_ERROR_STREAM(logger_, fail_reason_out);
    return false;
  }

  if (mission_plan.mission_items.empty())
  {
    ss << "Mission #" << id << " waypoints empty. Nothing to upload";
    fail_reason_out = ss.str();
    RCLCPP_ERROR_STREAM(logger_, fail_reason_out);
    return false;
  }

  mission_id_ = id;
  upload_attempts_ = 0;
  start_mission_upload(mission_plan);
  return true;
}
//}

/* stop_mission() method //{ */
bool MissionManager::stop_mission(std::string& fail_reason_out)
{
  std::scoped_lock lck(mutex_);
  std::stringstream ss;

  const auto orig_state = state_;
  // reset stuff
  plan_size_ = 0;
  current_waypoint_ = 0;
  // set the state to stopped to avoid any reupload attempts etc.
  update_state(state_t::canceled);

  // cancel any current mission upload to pixhawk if applicable
  if (orig_state == state_t::uploading)
  {
    const auto cancel_result = mission_->cancel_mission_upload();
    if (cancel_result != mavsdk::Mission::Result::Success)
    {
      ss << "cannot stop mission upload (" << to_string(cancel_result) << ")";
      fail_reason_out = ss.str();
      RCLCPP_ERROR_STREAM(logger_, "Failed to stop current mission: " << fail_reason_out);
      return false;
    }
  }

  // clear any currently uploaded mission
  const auto clear_result = mission_->clear_mission();
  if (clear_result != mavsdk::Mission::Result::Success)
  {
    ss << "cannot clear current mission (" << to_string(clear_result) << ")";
    fail_reason_out = ss.str();
    RCLCPP_ERROR_STREAM(logger_, "Failed to stop current mission: " << fail_reason_out);
    return false;
  }

  RCLCPP_INFO_STREAM(logger_, "Last mission stopped");
  return true;
}
//}

/* stop_mission_async() method //{ */
void MissionManager::stop_mission_async(const std::function<void(bool, const std::string&)> callback)
{
  std::thread([this, callback]
      {
        std::string reason;
        const bool succ = stop_mission(reason);
        callback(succ, reason);
      }).detach();
}
//}

/* getters //{ */
mission_state_t MissionManager::state()
{
  return fog_lib::get_mutexed(mutex_, state_);
}

uint32_t MissionManager::mission_id()
{
  return fog_lib::get_mutexed(mutex_, mission_id_);
}

int32_t MissionManager::mission_size()
{
  return fog_lib::get_mutexed(mutex_, plan_size_);
}

int32_t MissionManager::mission_waypoint()
{
  return fog_lib::get_mutexed(mutex_, current_waypoint_);
}
//}

// | --------------- private methods definitions -------------- |

/* start_mission_upload //{ */
void MissionManager::start_mission_upload(const mavsdk::Mission::MissionPlan& mission_plan)
{
  last_upload_attempt_time_ = clock_->now();
  mission_->upload_mission_async(mission_plan, [this, mission_plan](mavsdk::Mission::Result result)
      {
        // spawn a new thread to avoid blocking in the MavSDK
        // callback which will eventually cause a deadlock
        // of the MavSDK processing thread
        std::thread([this, result, mission_plan]
        {
          std::scoped_lock lck(mutex_);
          const double dur_s = (clock_->now() - last_upload_attempt_time_).seconds();
          // if mission upload succeeded, all is good and well in the world
          if (result == mavsdk::Mission::Result::Success)
          {
            RCLCPP_INFO_STREAM(logger_, "Mission #" << mission_id_ << " upload of " << mission_plan.mission_items.size() << " waypoints succeeded after " << dur_s << "s.");
            starting_attempts_ = 0;
            first_starting_attempt_time_ = clock_->now();
            start_mission(mission_plan);
          }
          // otherwise, check if we should retry or give up
          else
          {
            RCLCPP_INFO(logger_, "Mission #%u waypoints upload failed after %.2fs: %s", mission_id_, dur_s, to_string(result).c_str());
            if (upload_attempts_ < max_upload_attempts_)
            {
              // check if the state is still uploading - if not, the upload was probably cancelled in the meantime
              if (state_ == state_t::uploading)
              {
                upload_attempts_++;
                start_mission_upload(mission_plan);
              }
            }
            else
            {
              RCLCPP_WARN_STREAM(logger_, "Mission #" << mission_id_ << " upload failed too many times. Aborting mission.");
              update_state(state_t::aborted);
            }
          }
        }).detach();
      }
    );

  RCLCPP_INFO(logger_, "Started mission #%u upload (attempt %d/%d).", mission_id_, upload_attempts_+1, max_upload_attempts_+1);
  update_state(state_t::uploading);
}
//}

/* start_mission //{ */
void MissionManager::start_mission(const mavsdk::Mission::MissionPlan& mission_plan)
{
  plan_size_ = 0;
  current_waypoint_ = 0;

  mission_->start_mission_async([this, mission_plan](mavsdk::Mission::Result result)
      {
        // spawn a new thread to avoid blocking in the MavSDK
        // callback which will eventually cause a deadlock
        // of the MavSDK processing thread
        std::thread([this, result, mission_plan]
        {
          std::scoped_lock lck(mutex_);
          const rclcpp::Duration starting_dur = clock_->now() - first_starting_attempt_time_;
          // if mission start succeeded, all is good and well in the world
          if (result == mavsdk::Mission::Result::Success)
          {
            RCLCPP_INFO(logger_, "Mission #%u successfully started after %.2fs.", mission_id_, starting_dur.seconds());
            update_state(state_t::in_progress);
            publish_plan(mission_plan);
          }
          // otherwise, check if we should retry or give up
          else
          {
            RCLCPP_WARN(logger_, "Calling mission #%u start rejected after %.2fs: %s", mission_id_, starting_dur.seconds(), to_string(result).c_str());
            if (starting_dur < starting_timeout_)
            {
              // check if the state is still starting
              // if not, the mission was probably cancelled in the meantime, do not retry starting
              // if yes, retry starting
              if (state_ == state_t::starting)
              {
                starting_attempts_++;
                start_mission(mission_plan);
              }
            }
            else
            {
              RCLCPP_ERROR_STREAM(logger_, "Calling mission #" << mission_id_ << " start timed out (took " << starting_dur.seconds() << "s/" << starting_timeout_.seconds() << "s). Aborting mission.");
              update_state(state_t::aborted);
            }
          }
        }).detach();
      }
    );

  RCLCPP_INFO_STREAM(logger_, "Called mission #" << mission_id_ << " start (attempt " << starting_attempts_+1 << ").");
  update_state(state_t::starting);
}
//}

/* publish_plan //{ */
void MissionManager::publish_plan(const mavsdk::Mission::MissionPlan& mission_plan)
{
  fog_msgs::msg::MissionPlan::UniquePtr msg = std::make_unique<fog_msgs::msg::MissionPlan>();
  msg->mission_items.reserve(mission_plan.mission_items.size());
  for (const auto& it : mission_plan.mission_items)
  {
    fog_msgs::msg::MissionItem it_msg;
    it_msg.latitude_deg = it.latitude_deg;
    it_msg.longitude_deg = it.longitude_deg;
    it_msg.relative_altitude_m = it.relative_altitude_m;
    it_msg.speed_m_s = it.speed_m_s;
    it_msg.is_fly_through = it.is_fly_through;
    it_msg.gimbal_pitch_deg = it.gimbal_pitch_deg;
    it_msg.gimbal_yaw_deg = it.gimbal_yaw_deg;
    it_msg.loiter_time_s = it.loiter_time_s;
    it_msg.camera_photo_interval_s = it.camera_photo_interval_s;
    it_msg.acceptance_radius_m = it.acceptance_radius_m;
    it_msg.yaw_deg = it.yaw_deg;
    it_msg.camera_action = (int)it.camera_action;
    msg->mission_items.push_back(it_msg);
  }
  plan_pub_->publish(std::move(msg));
}
//}

/* progress_callback //{ */
void MissionManager::progress_callback(const mavsdk::Mission::MissionProgress& progress)
{
  // spawn a new thread to avoid blocking in the MavSDK
  // callback which will eventually cause a deadlock
  // of the MavSDK processing thread
  std::thread([this, &progress]
  {
    std::scoped_lock lck(mutex_);

    // if no mission is in progress, don't print or update anything to avoid spamming garbage
    if (state_ != state_t::in_progress)
      return;

    plan_size_ = progress.total;
    current_waypoint_ = progress.current;

    const float percent = progress.total == 0 ? 100 : progress.current/float(progress.total)*100.0f;
    if (progress.current == -1)
      RCLCPP_INFO(logger_, "Last mission canceled (waypoint %d/%d).", progress.current, progress.total);
    else if (progress.total == 0)
      RCLCPP_INFO(logger_, "Current mission #%u is empty (waypoint %d/%d).", mission_id_, progress.current, progress.total);
    else
      RCLCPP_INFO(logger_, "Current mission #%u waypoint: %d/%d (%.1f%%).", mission_id_, progress.current, progress.total, percent);

    if (plan_size_ == current_waypoint_)
    {
      RCLCPP_INFO_STREAM(logger_, "Mission #" << mission_id_ << " finished.");
      update_state(state_t::finished);
    }
  }).detach();
}
//}

/* set_state_update_function //{ */
void MissionManager::subscribe_state_update(const state_update_cbk_t& func)
{
  state_update_cbk_ = func;
}
//}

/* update_state //{ */
void MissionManager::update_state(const state_t new_state)
{
  if (state_ != new_state)
  {
    /* RCLCPP_INFO_STREAM(logger_, "new mission state: " << to_string(new_state)); */
    state_ = new_state;
    if (state_update_cbk_)
      state_update_cbk_();
  }
}
//}
