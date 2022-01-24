#include "control_interface/mission_manager.h"

#include <fog_lib/mutex_utils.h>

#include <memory>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

using namespace control_interface;

/* constructor //{ */
MissionManager::MissionManager()
  : logger_(rclcpp::get_logger("PLEASE INITIALIZE MissionManager"))
{
}

MissionManager::MissionManager(const unsigned max_attempts, std::shared_ptr<mavsdk::System> system, const rclcpp::Logger& logger, rclcpp::Clock::SharedPtr clock)
  : max_attempts_(max_attempts), mission_(std::make_unique<mavsdk::Mission>(system)), logger_(logger), clock_(clock)
{
    // register some callbacks
    const mavsdk::Mission::MissionProgressCallback progress_cbk = std::bind(&MissionManager::progressCallback, this, std::placeholders::_1);
    mission_->subscribe_mission_progress(progress_cbk);
}
//}

/* startMissionUpload //{ */
bool MissionManager::startMissionUpload(const mavsdk::Mission::MissionPlan& mission_plan)
{
  if (mission_plan.mission_items.empty())
  {
    RCLCPP_ERROR(logger_, "Mission waypoints empty. Nothing to upload");
    return false;
  }

  attempt_start_time_ = clock_->now();
  mission_->upload_mission_async(mission_plan, [this](mavsdk::Mission::Result result)
      {
        // spawn a new thread to avoid blocking in the MavSDK
        // callback which will eventually cause a deadlock
        // of the MavSDK processing thread
        std::thread([this, result]
        {
          std::scoped_lock lck(mutex_);
          const double dur_s = (clock_->now() - attempt_start_time_).seconds();
          // if mission upload succeeded, all is good and well in the world
          if (result == mavsdk::Mission::Result::Success)
          {
            RCLCPP_INFO(logger_, "Mission upload of %ld waypoints succeeded after %.2fs.", mission_plan_.mission_items.size(), dur_s);
            attempts_ = 0;
            startMission();
          }
          // otherwise, check if we should retry or give up
          else
          {
            RCLCPP_INFO(logger_, "Mission waypoints upload failed after %.2fs: %s", dur_s, to_string(result).c_str());
            if (attempts_ < max_attempts_)
            {
              // check if the state is still uploading - if not, the upload was probably cancelled in the meantime
              if (state_ == state_t::uploading)
              {
                attempts_++;
                startMissionUpload(mission_plan_);
              }
            }
            else
            {
              state_ = state_t::finished;
              RCLCPP_WARN(logger_, "Mission upload failed too many times. Scrapping mission.");
            }
          }
        }).detach();
      }
    );

  state_ = state_t::uploading;
  RCLCPP_INFO(logger_, "Started mission upload (attempt %d/%d).", attempts_+1, max_attempts_+1);
  return true;
}
//}

/* startMission //{ */
bool MissionManager::startMission()
{
  plan_size_ = 0;
  current_waypoint_ = 0;
  attempt_start_time_ = clock_->now();
  mission_->start_mission_async([this](mavsdk::Mission::Result result)
      {
        // spawn a new thread to avoid blocking in the MavSDK
        // callback which will eventually cause a deadlock
        // of the MavSDK processing thread
        std::thread([this, result]
        {
          std::scoped_lock lck(mutex_);
          const double dur_s = (clock_->now() - attempt_start_time_).seconds();
          // if mission start succeeded, all is good and well in the world
          if (result == mavsdk::Mission::Result::Success)
          {
            RCLCPP_INFO(logger_, "Mission successfully started after %.2fs.", dur_s);
            attempts_ = 0;
            state_ = state_t::in_progress;
          }
          // otherwise, check if we should retry or give up
          else
          {
            RCLCPP_ERROR(logger_, "Calling mission start rejected after %.2fs: %s", dur_s, to_string(result).c_str());
            if (attempts_ < max_attempts_)
            {
              // check if the state is still starting - if not, the mission was probably cancelled in the meantime
              if (state_ == state_t::starting)
              {
                attempts_++;
                startMission();
              }
            }
            else
            {
              state_ = state_t::finished;
              RCLCPP_WARN(logger_, "Calling mission start failed too many times. Scrapping mission.");
            }
          }
        }).detach();
      }
    );

  state_ = state_t::starting;
  RCLCPP_INFO(logger_, "Called mission start (attempt %d/%d).", attempts_+1, max_attempts_+1);
  return true;
}
//}

/* stopMission //{ */
bool MissionManager::stopMission(std::string& fail_reason_out)
{
  std::stringstream ss;

  // clear and reset stuff
  mission_plan_.mission_items.clear();
  plan_size_ = 0;
  current_waypoint_ = 0;

  // cancel any current mission upload to pixhawk if applicable
  if (state_ == state_t::uploading)
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

  RCLCPP_INFO(logger_, "Current mission stopped");
  return true;
}
//}

/* progressCallback //{ */
void MissionManager::progressCallback(const mavsdk::Mission::MissionProgress& progress)
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
      RCLCPP_INFO(logger_, "Current mission cancelled (waypoint %d/%d).", progress.current, progress.total);
    else if (progress.total == 0)
      RCLCPP_INFO(logger_, "Current mission is empty (waypoint %d/%d).", progress.current, progress.total);
    else
      RCLCPP_INFO(logger_, "Current mission waypoint: %d/%d (%.1f%%).", progress.current, progress.total, percent);
  }).detach();
}
//}

mission_state_t MissionManager::state()
{
  return fog_lib::get_mutexed(mutex_, state_);
}

int32_t MissionManager::mission_size()
{
  return plan_size_;
}

int32_t MissionManager::mission_waypoint()
{
  return current_waypoint_;
}
