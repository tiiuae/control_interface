#ifndef UTILS_H
#define UTILS_H

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

namespace control_interface
{
  std::string to_string(const mavsdk::Action::Result result);
  std::string to_string(const mavsdk::Mission::Result result);
  std::string to_string(const mavsdk::Param::Result result);
  std::string to_string(const mavsdk::ConnectionResult result);
  std::string to_string(const mavsdk::Telemetry::FlightMode mode);
  std::string to_string(const mavsdk::Telemetry::LandedState land_state);
}

#endif // UTILS_H
