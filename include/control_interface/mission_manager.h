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
    mission_state_t state();
    int32_t mission_size();
    int32_t mission_waypoint();

  private:
    using state_t = mission_state_t;

    unsigned max_attempts_ = 5;
    std::unique_ptr<mavsdk::Mission> mission_;
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;

    std::recursive_mutex mutex_;
    state_t state_;
    unsigned attempts_;
    mavsdk::Mission::MissionPlan mission_plan_; // this buffer is used for repeated attempts at mission uploads
    rclcpp::Time attempt_start_time_;

    // set by the MavSDK progressCallback
    int32_t plan_size_ = 0;
    int32_t current_waypoint_ = 0;

    bool startMissionUpload(const mavsdk::Mission::MissionPlan& mission_plan);
    bool startMission();
    bool stopMission(std::string& fail_reason_out);

    void progressCallback(const mavsdk::Mission::MissionProgress& progress);
    
  };

}

#endif // MISSION_MANAGER_H
