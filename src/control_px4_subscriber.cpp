// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <px4_msgs/msg/mission_result.hpp>
using std::placeholders::_1;

namespace control_interface
{

class MissionResultRelayer : public rclcpp::Node
{
public:
  MissionResultRelayer(rclcpp::NodeOptions options)
  : Node("mission_result_relayer", options)
  {
    mission_result_subscriber_ = this->create_subscription<px4_msgs::msg::MissionResult>(
      "~/mission_result_in", rclcpp::SystemDefaultsQoS(), 
      std::bind(&MissionResultRelayer::topic_callback, this, _1));

    mission_result_publisher_ = this->create_publisher<px4_msgs::msg::MissionResult>(
      "~/mission_result_out", rclcpp::QoS(10).transient_local());
  }

private:
  void topic_callback(const px4_msgs::msg::MissionResult::UniquePtr msg) const
  {
    if (msg->seq_reached != -1){
      RCLCPP_INFO(this->get_logger(), "[%s]:[%s] Instance: %d - Sequence: [%d - %d - %d]", 
        this->get_name(), msg->finished ? "Finished" : "Ongoing", msg->instance_count,
        msg->seq_reached, msg->seq_current, msg->seq_total);
    }
    mission_result_publisher_->publish(*msg);
  }
  rclcpp::Subscription<px4_msgs::msg::MissionResult>::SharedPtr mission_result_subscriber_;
  rclcpp::Publisher<px4_msgs::msg::MissionResult>::SharedPtr mission_result_publisher_;

};

}  // namespace control_interface
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(control_interface::MissionResultRelayer)
