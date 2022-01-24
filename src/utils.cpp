#include "control_interface/utils.h"

using namespace control_interface;

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
