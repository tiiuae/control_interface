#pragma once

#include <cassert>
#include <fog_msgs/msg/control_interface_vehicle_state.hpp>
#include <fog_msgs/msg/control_interface_mission_state.hpp>

namespace control_interface
{
#define ERR_MSG "Inconsistent message received, please rebuild the package that crashed!"

  /* vehicle_state_t enumeration type //{ */
  
  enum struct vehicle_state_t
  {
    invalid,
    not_connected,
    not_ready,
    arming_ready,
    takeoff_ready,
    taking_off,
    autonomous_flight,
    manual_flight,
    landing,
  };
  
  static inline vehicle_state_t to_enum(const fog_msgs::msg::ControlInterfaceVehicleState msg)
  {
    switch (msg.state)
    {
      case fog_msgs::msg::ControlInterfaceVehicleState::NOT_CONNECTED:       return vehicle_state_t::not_connected;
      case fog_msgs::msg::ControlInterfaceVehicleState::NOT_READY:           return vehicle_state_t::not_ready;
      case fog_msgs::msg::ControlInterfaceVehicleState::ARMING_READY:        return vehicle_state_t::arming_ready;
      case fog_msgs::msg::ControlInterfaceVehicleState::TAKEOFF_READY:       return vehicle_state_t::takeoff_ready;
      case fog_msgs::msg::ControlInterfaceVehicleState::TAKING_OFF:          return vehicle_state_t::taking_off;
      case fog_msgs::msg::ControlInterfaceVehicleState::AUTONOMOUS_FLIGHT:   return vehicle_state_t::autonomous_flight;
      case fog_msgs::msg::ControlInterfaceVehicleState::MANUAL_FLIGHT:       return vehicle_state_t::manual_flight;
      case fog_msgs::msg::ControlInterfaceVehicleState::LANDING:             return vehicle_state_t::landing;
      default:                                                               assert(false && ERR_MSG); return vehicle_state_t::invalid;
    }
  }
  
  static inline fog_msgs::msg::ControlInterfaceVehicleState to_msg(const vehicle_state_t enum_val)
  {
    fog_msgs::msg::ControlInterfaceVehicleState msg;
    switch (enum_val)
    {
      case vehicle_state_t::not_connected:       msg.state = fog_msgs::msg::ControlInterfaceVehicleState::NOT_CONNECTED; break;
      case vehicle_state_t::not_ready:           msg.state = fog_msgs::msg::ControlInterfaceVehicleState::NOT_READY; break;
      case vehicle_state_t::arming_ready:        msg.state = fog_msgs::msg::ControlInterfaceVehicleState::ARMING_READY; break;
      case vehicle_state_t::takeoff_ready:       msg.state = fog_msgs::msg::ControlInterfaceVehicleState::TAKEOFF_READY; break;
      case vehicle_state_t::taking_off:          msg.state = fog_msgs::msg::ControlInterfaceVehicleState::TAKING_OFF; break;
      case vehicle_state_t::autonomous_flight:   msg.state = fog_msgs::msg::ControlInterfaceVehicleState::AUTONOMOUS_FLIGHT; break;
      case vehicle_state_t::manual_flight:       msg.state = fog_msgs::msg::ControlInterfaceVehicleState::MANUAL_FLIGHT; break;
      case vehicle_state_t::landing:             msg.state = fog_msgs::msg::ControlInterfaceVehicleState::LANDING; break;
      default:                                   assert(false && ERR_MSG); msg.state = fog_msgs::msg::ControlInterfaceVehicleState::INVALID; break;
    }
    return msg;
  }

  static inline std::string to_string(const vehicle_state_t enum_val)
  {
    switch (enum_val)
    {
      case vehicle_state_t::not_connected:       return "not_connected";
      case vehicle_state_t::not_ready:           return "not_ready";
      case vehicle_state_t::arming_ready:        return "arming_ready";
      case vehicle_state_t::takeoff_ready:       return "takeoff_ready";
      case vehicle_state_t::taking_off:          return "taking_off";
      case vehicle_state_t::autonomous_flight:   return "autonomous_flight";
      case vehicle_state_t::manual_flight:       return "manual_flight";
      case vehicle_state_t::landing:             return "landing";
      default:                                   assert(false && ERR_MSG); return "invalid";
    }
  }
  
  //}

  /* mission_state_t enumeration type //{ */
  
  enum struct mission_state_t
  {
    invalid,
    uploading,
    starting,
    in_progress,
    finished,
  };
  
  static inline mission_state_t to_enum(const fog_msgs::msg::ControlInterfaceMissionState msg)
  {
    switch (msg.state)
    {
      case fog_msgs::msg::ControlInterfaceMissionState::UPLOADING:    return mission_state_t::uploading;
      case fog_msgs::msg::ControlInterfaceMissionState::STARTING:     return mission_state_t::starting;
      case fog_msgs::msg::ControlInterfaceMissionState::IN_PROGRESS:  return mission_state_t::in_progress;
      case fog_msgs::msg::ControlInterfaceMissionState::FINISHED:     return mission_state_t::finished;
      default:                                                        assert(false && ERR_MSG); return mission_state_t::invalid;
    }
  }
  
  static inline fog_msgs::msg::ControlInterfaceMissionState to_msg(const mission_state_t enum_val)
  {
    fog_msgs::msg::ControlInterfaceMissionState msg;
    switch (enum_val)
    {
      case mission_state_t::uploading:   msg.state = fog_msgs::msg::ControlInterfaceMissionState::UPLOADING; break;
      case mission_state_t::starting:    msg.state = fog_msgs::msg::ControlInterfaceMissionState::STARTING; break;
      case mission_state_t::in_progress: msg.state = fog_msgs::msg::ControlInterfaceMissionState::IN_PROGRESS; break;
      case mission_state_t::finished:    msg.state = fog_msgs::msg::ControlInterfaceMissionState::FINISHED; break;
      default:                           assert(false && ERR_MSG); msg.state = fog_msgs::msg::ControlInterfaceMissionState::INVALID; break;
    }
    return msg;
  }

  static inline std::string to_string(const mission_state_t enum_val)
  {
    switch (enum_val)
    {
      case mission_state_t::uploading:    return "uploading";
      case mission_state_t::starting:     return "starting";
      case mission_state_t::in_progress:  return "in_progress";
      case mission_state_t::finished:     return "finished";
      default:                            assert(false && ERR_MSG); return "invalid";
    }
  }
  
  //}
}
