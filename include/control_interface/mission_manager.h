#ifndef MISSION_MANAGER_H
#define MISSION_MANAGER_H

/* includes //{ */

#include "control_interface/enums.h"
#include "control_interface/utils.h"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mission/mission.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <mutex>

//}

namespace control_interface
{

  class MissionManager
  {
  public:
    MissionManager();
    MissionManager(const unsigned max_attempts, std::shared_ptr<mavsdk::System> system, const rclcpp::Logger& logger, rclcpp::Clock::SharedPtr clock);

    // doesn't block (the uploading & starting of the mission is asynchronous)
    bool new_mission(const mavsdk::Mission::MissionPlan& mission_plan, const uint32_t id, std::string& fail_reason_out);
    // blocks until either the mission is stopped or it fails
    bool stop_mission(std::string& fail_reason_out);
    // some getters
    mission_state_t state();
    uint32_t mission_id();
    int32_t mission_size();
    int32_t mission_waypoint();
    std::recursive_mutex& mutex();

  private:
    using state_t = mission_state_t;
    std::recursive_mutex mutex_;

    // state of the current mission
    state_t state_ = state_t::finished;
    uint32_t mission_id_ = 0;

    // the MavSDK mission interface
    std::unique_ptr<mavsdk::Mission> mission_;

    // current number of retry attempts (uploads and mission starts)
    unsigned attempts_;
    // maximal number of retry attempts (parameter)
    unsigned max_attempts_ = 5;
    // time of the last retry attempt
    rclcpp::Time attempt_start_time_;

    // for printing
    rclcpp::Logger logger_;
    // for time-related stuff
    rclcpp::Clock::SharedPtr clock_;

    // set by the MavSDK progressCallback
    int32_t plan_size_ = 0;
    int32_t current_waypoint_ = 0;

    bool startMissionUpload(const mavsdk::Mission::MissionPlan& mission_plan);
    bool startMission();

    void progressCallback(const mavsdk::Mission::MissionProgress& progress);
    
  };

}

#endif // MISSION_MANAGER_H
