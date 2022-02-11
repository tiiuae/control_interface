#include "control_interface/mission_manager.h"

#include <fog_lib/mutex_utils.h>

#include <memory>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

using namespace control_interface;

// | --------------- public methods definitions --------------- |

/* constructor //{ */
MissionManager::MissionManager(const unsigned max_upload_attempts, const rclcpp::Duration& starting_timeout, std::shared_ptr<mavsdk::System> system, const rclcpp::Logger& logger, rclcpp::Clock::SharedPtr clock)
  : mission_(std::make_unique<mavsdk::Mission>(system)), max_upload_attempts_(max_upload_attempts), starting_timeout_(starting_timeout), logger_(logger), clock_(clock)
{
    // register some callbacks
    const mavsdk::Mission::MissionProgressCallback progress_cbk = std::bind(&MissionManager::progress_callback, this, std::placeholders::_1);
    mission_->subscribe_mission_progress(progress_cbk);
}
//}

/* new_mission() method //{ */
bool MissionManager::new_mission(const mavsdk::Mission::MissionPlan& mission_plan, const uint32_t id, std::string& fail_reason_out)
{
  std::scoped_lock lck(mutex);
  std::stringstream ss;

  if (state_ != state_t::finished)
  {
    ss << "Last mission #" << mission_id_ << " not cancelled (mission " << to_string(state_) << "), cannot upload new mission.";
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
  return start_mission_upload(mission_plan);
}
//}

/* stop_mission() method //{ */
bool MissionManager::stop_mission(std::string& fail_reason_out)
{
  std::scoped_lock lck(mutex);
  std::stringstream ss;

  const auto orig_state = state_;
  // reset stuff
  plan_size_ = 0;
  current_waypoint_ = 0;
  // set the state to finished to avoid any reupload attempts etc.
  state_ = state_t::finished;

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

/* getters //{ */
mission_state_t MissionManager::state()
{
  return fog_lib::get_mutexed(mutex, state_);
}

uint32_t MissionManager::mission_id()
{
  return fog_lib::get_mutexed(mutex, mission_id_);
}

int32_t MissionManager::mission_size()
{
  return fog_lib::get_mutexed(mutex, plan_size_);
}

int32_t MissionManager::mission_waypoint()
{
  return fog_lib::get_mutexed(mutex, current_waypoint_);
}
//}

// | --------------- private methods definitions -------------- |

/* startMissionUpload //{ */
bool MissionManager::start_mission_upload(const mavsdk::Mission::MissionPlan& mission_plan)
{
  last_upload_attempt_time_ = clock_->now();
  mission_->upload_mission_async(mission_plan, [this, mission_plan](mavsdk::Mission::Result result)
      {
        // spawn a new thread to avoid blocking in the MavSDK
        // callback which will eventually cause a deadlock
        // of the MavSDK processing thread
        std::thread([this, result, mission_plan]
        {
          std::scoped_lock lck(mutex);
          const double dur_s = (clock_->now() - last_upload_attempt_time_).seconds();
          // if mission upload succeeded, all is good and well in the world
          if (result == mavsdk::Mission::Result::Success)
          {
            RCLCPP_INFO_STREAM(logger_, "Mission #" << mission_id_ << " upload of " << mission_plan.mission_items.size() << " waypoints succeeded after " << dur_s << "s.");
            starting_attempts_ = 0;
            first_starting_attempt_time_ = clock_->now();
            start_mission();
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
              state_ = state_t::finished;
              RCLCPP_WARN_STREAM(logger_, "Mission #" << mission_id_ << " upload failed too many times. Scrapping mission.");
            }
          }
        }).detach();
      }
    );

  state_ = state_t::uploading;
  RCLCPP_INFO(logger_, "Started mission #%u upload (attempt %d/%d).", mission_id_, upload_attempts_+1, max_upload_attempts_+1);
  return true;
}
//}

/* startMission //{ */
bool MissionManager::start_mission()
{
  plan_size_ = 0;
  current_waypoint_ = 0;

  mission_->start_mission_async([this](mavsdk::Mission::Result result)
      {
        // spawn a new thread to avoid blocking in the MavSDK
        // callback which will eventually cause a deadlock
        // of the MavSDK processing thread
        std::thread([this, result]
        {
          std::scoped_lock lck(mutex);
          const rclcpp::Duration starting_dur = clock_->now() - first_starting_attempt_time_;
          // if mission start succeeded, all is good and well in the world
          if (result == mavsdk::Mission::Result::Success)
          {
            RCLCPP_INFO(logger_, "Mission #%u successfully started after %.2fs.", mission_id_, starting_dur.seconds());
            state_ = state_t::in_progress;
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
                start_mission();
              }
            }
            else
            {
              state_ = state_t::finished;
              RCLCPP_ERROR_STREAM(logger_, "Calling mission #" << mission_id_ << " start timed out (took " << starting_dur.seconds() << "s/" << starting_timeout_.seconds() << "s). Scrapping mission.");
            }
          }
        }).detach();
      }
    );

  state_ = state_t::starting;
  RCLCPP_INFO_STREAM(logger_, "Called mission #" << mission_id_ << " start (attempt " << starting_attempts_+1 << ").");
  return true;
}
//}

/* progressCallback //{ */
void MissionManager::progress_callback(const mavsdk::Mission::MissionProgress& progress)
{
  // spawn a new thread to avoid blocking in the MavSDK
  // callback which will eventually cause a deadlock
  // of the MavSDK processing thread
  std::thread([this, &progress]
  {
    std::scoped_lock lck(mutex);

    // if no mission is in progress, don't print or update anything to avoid spamming garbage
    if (state_ != state_t::in_progress)
      return;

    plan_size_ = progress.total;
    current_waypoint_ = progress.current;

    const float percent = progress.total == 0 ? 100 : progress.current/float(progress.total)*100.0f;
    if (progress.current == -1)
      RCLCPP_INFO(logger_, "Last mission cancelled (waypoint %d/%d).", progress.current, progress.total);
    else if (progress.total == 0)
      RCLCPP_INFO(logger_, "Current mission #%u is empty (waypoint %d/%d).", mission_id_, progress.current, progress.total);
    else
      RCLCPP_INFO(logger_, "Current mission #%u waypoint: %d/%d (%.1f%%).", mission_id_, progress.current, progress.total, percent);

    if (plan_size_ == current_waypoint_)
    {
      RCLCPP_INFO_STREAM(logger_, "Mission #" << mission_id_ << " finished.");
      state_ = state_t::finished;
    }
  }).detach();
}
//}
