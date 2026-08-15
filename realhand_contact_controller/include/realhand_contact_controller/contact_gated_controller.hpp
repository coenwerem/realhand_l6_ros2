// Copyright 2026 Clinton Enwerem
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

#ifndef REALHAND_CONTACT_CONTROLLER__CONTACT_GATED_CONTROLLER_HPP_
#define REALHAND_CONTACT_CONTROLLER__CONTACT_GATED_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "contact_gated_controller_parameters.hpp"

namespace realhand_contact_controller
{

// Closes each finger toward a commanded target and freezes it the moment its
// tactile pad reports contact. Fingers that reach the target without ever
// touching anything are reported as missed, so a caller can tell a grip
// from a close on air.
//
// Topics, all under the controller node namespace.
//   ~/close_to       sensor_msgs/JointState in, target per command joint,
//                    starts a contact gated close
//   ~/open           std_msgs/Bool in, true moves every joint to
//                    open_position without gating
//   ~/contact_state  std_msgs/Int32MultiArray out, one code per finger
//                    0 no contact, 2 contact sensed and still moving,
//                    1 latched on contact (gripped), 3 latched at target
//                    with no contact seen (missed)
//   ~/finger_force   std_msgs/Float64MultiArray out, summed pad force
class ContactGatedController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Contact codes published on ~/contact_state.
  static constexpr int CONTACT_NONE = 0;
  static constexpr int CONTACT_LATCHED = 1;
  static constexpr int CONTACT_SENSING = 2;
  static constexpr int CONTACT_MISSED = 3;

private:
  enum class State {IDLE, CLOSE, HOLD};

  int command_index(const std::string & interface_name) const;
  void publish_diagnostics(const std::vector<bool> & contact);

  std::shared_ptr<contact_gated_controller::ParamListener> param_listener_;
  contact_gated_controller::Params params_;

  std::size_t n_fingers_{0};
  std::vector<int> cmd_idx_;             // command_joints index to loaned interface
  std::vector<int> flexion_cmd_idx_;     // finger to loaned interface
  std::vector<int> flexion_slot_;        // finger to command_joints slot
  int thumb_opp_slot_{-1};
  std::vector<int> force_state_idx_;     // finger to loaned state interface

  std::vector<double> cmd_pos_;
  std::vector<double> target_pos_;
  std::vector<double> finger_force_;
  std::vector<bool> latched_;
  std::vector<bool> contacted_;
  std::vector<bool> contact_;
  State state_{State::IDLE};
  bool gated_close_{true};

  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>> target_buf_;
  std::atomic<bool> start_close_{false};
  std::atomic<bool> request_open_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr close_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr open_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr contact_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_pub_;
  using ContactPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Int32MultiArray>;
  using ForcePublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;
  std::unique_ptr<ContactPublisher> rt_contact_pub_;
  std::unique_ptr<ForcePublisher> rt_force_pub_;
};

}  // namespace realhand_contact_controller

#endif  // REALHAND_CONTACT_CONTROLLER__CONTACT_GATED_CONTROLLER_HPP_
