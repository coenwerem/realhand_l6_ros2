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

#include "realhand_contact_controller/contact_gated_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{

// Move cur toward tgt by at most step.
double step_toward(double cur, double tgt, double step)
{
  const double d = tgt - cur;
  if (std::abs(d) <= step) {return tgt;}
  return cur + (d > 0.0 ? step : -step);
}

}  // namespace

namespace realhand_contact_controller
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;
using controller_interface::return_type;

CallbackReturn ContactGatedController::on_init()
{
  try {
    param_listener_ = std::make_shared<contact_gated_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter setup failed, %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn ContactGatedController::on_configure(const rclcpp_lifecycle::State &)
{
  params_ = param_listener_->get_params();
  auto logger = get_node()->get_logger();

  n_fingers_ = params_.finger_names.size();
  if (params_.flexion_joints.size() != n_fingers_) {
    RCLCPP_ERROR(
      logger, "finger_names has %zu entries and flexion_joints has %zu, they must match.",
      n_fingers_, params_.flexion_joints.size());
    return CallbackReturn::ERROR;
  }
  const auto & cj = params_.command_joints;
  for (const auto & fj : params_.flexion_joints) {
    if (std::find(cj.begin(), cj.end(), fj) == cj.end()) {
      RCLCPP_ERROR(logger, "Flexion joint '%s' is not in command_joints.", fj.c_str());
      return CallbackReturn::ERROR;
    }
  }
  if (!params_.thumb_opposition_joint.empty() &&
    std::find(cj.begin(), cj.end(), params_.thumb_opposition_joint) == cj.end())
  {
    RCLCPP_ERROR(
      logger, "thumb_opposition_joint '%s' is not in command_joints.",
      params_.thumb_opposition_joint.c_str());
    return CallbackReturn::ERROR;
  }

  const std::size_t n_cmd = cj.size();
  cmd_pos_.assign(n_cmd, params_.open_position);
  target_pos_.assign(n_cmd, params_.open_position);
  finger_force_.assign(n_fingers_, 0.0);
  latched_.assign(n_fingers_, false);
  contacted_.assign(n_fingers_, false);
  contact_.assign(n_fingers_, false);

  // Static slot lookups so update() does no string work.
  flexion_slot_.assign(n_fingers_, -1);
  for (std::size_t f = 0; f < n_fingers_; ++f) {
    const auto it = std::find(cj.begin(), cj.end(), params_.flexion_joints[f]);
    flexion_slot_[f] = static_cast<int>(std::distance(cj.begin(), it));
  }
  thumb_opp_slot_ = -1;
  if (!params_.thumb_opposition_joint.empty()) {
    const auto it = std::find(cj.begin(), cj.end(), params_.thumb_opposition_joint);
    thumb_opp_slot_ = static_cast<int>(std::distance(cj.begin(), it));
  }

  auto node = get_node();
  close_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "~/close_to", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<sensor_msgs::msg::JointState> msg) {
      target_buf_.writeFromNonRT(msg);
      start_close_.store(true);
    });
  open_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "~/open", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<std_msgs::msg::Bool> msg) {
      if (msg->data) {request_open_.store(true);}
    });

  contact_pub_ = node->create_publisher<std_msgs::msg::Int32MultiArray>(
    "~/contact_state", rclcpp::SystemDefaultsQoS());
  force_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/finger_force", rclcpp::SystemDefaultsQoS());
  rt_contact_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Int32MultiArray>>(contact_pub_);
  rt_force_pub_ =
    std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(force_pub_);
  // Size the message buffers once so update() assigns in place.
  rt_contact_pub_->msg_.data.assign(n_fingers_, 0);
  rt_force_pub_->msg_.data.assign(n_fingers_, 0.0);

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration ContactGatedController::command_interface_configuration() const
{
  InterfaceConfiguration cfg;
  if (params_.monitor_only) {
    cfg.type = interface_configuration_type::NONE;
    return cfg;
  }
  cfg.type = interface_configuration_type::INDIVIDUAL;
  for (const auto & j : params_.command_joints) {
    cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
  }
  return cfg;
}

InterfaceConfiguration ContactGatedController::state_interface_configuration() const
{
  InterfaceConfiguration cfg;
  cfg.type = interface_configuration_type::INDIVIDUAL;
  for (const auto & f : params_.finger_names) {
    cfg.names.push_back(params_.sensor_prefix + f + "/force");
  }
  return cfg;
}

int ContactGatedController::command_index(const std::string & interface_name) const
{
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (command_interfaces_[i].get_name() == interface_name) {return static_cast<int>(i);}
  }
  return -1;
}

CallbackReturn ContactGatedController::on_activate(const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();
  const std::size_t n_cmd = params_.command_joints.size();

  if (!params_.monitor_only) {
    cmd_idx_.assign(n_cmd, -1);
    for (std::size_t j = 0; j < n_cmd; ++j) {
      cmd_idx_[j] =
        command_index(params_.command_joints[j] + "/" + hardware_interface::HW_IF_POSITION);
      if (cmd_idx_[j] < 0) {
        RCLCPP_ERROR(
          logger, "Missing command interface for joint '%s'.", params_.command_joints[j].c_str());
        return CallbackReturn::ERROR;
      }
    }
    flexion_cmd_idx_.assign(n_fingers_, -1);
    for (std::size_t f = 0; f < n_fingers_; ++f) {
      flexion_cmd_idx_[f] = cmd_idx_[flexion_slot_[f]];
    }
  }

  force_state_idx_.assign(n_fingers_, -1);
  for (std::size_t f = 0; f < n_fingers_; ++f) {
    const std::string name = params_.sensor_prefix + params_.finger_names[f] + "/force";
    for (std::size_t s = 0; s < state_interfaces_.size(); ++s) {
      if (state_interfaces_[s].get_name() == name) {
        force_state_idx_[f] = static_cast<int>(s);
        break;
      }
    }
    if (force_state_idx_[f] < 0) {
      RCLCPP_ERROR(logger, "Missing tactile state interface '%s'.", name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // Start from the current command so activation does not move the hand.
  if (!params_.monitor_only) {
    for (std::size_t j = 0; j < n_cmd; ++j) {
      const auto v = command_interfaces_[cmd_idx_[j]].get_optional();
      cmd_pos_[j] = (v && std::isfinite(*v)) ? *v : params_.open_position;
      target_pos_[j] = cmd_pos_[j];
    }
  }
  std::fill(latched_.begin(), latched_.end(), false);
  std::fill(contacted_.begin(), contacted_.end(), false);
  state_ = State::IDLE;
  gated_close_ = true;
  start_close_.store(false);
  request_open_.store(false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ContactGatedController::on_deactivate(const rclcpp_lifecycle::State &)
{
  state_ = State::IDLE;
  return CallbackReturn::SUCCESS;
}

return_type ContactGatedController::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  // 1. Per finger contact from the summed pad force the hardware exports.
  for (std::size_t f = 0; f < n_fingers_; ++f) {
    const auto v = state_interfaces_[force_state_idx_[f]].get_optional();
    const double force = (v && std::isfinite(*v)) ? *v : 0.0;
    finger_force_[f] = force;
    contact_[f] = force >= params_.contact_threshold;
  }

  if (params_.monitor_only) {
    publish_diagnostics(contact_);
    return return_type::OK;
  }

  // 2. Requests. Open wins over close. gated_close_ persists across cycles
  //    so an open motion stays ungated until the next close request.
  if (request_open_.exchange(false)) {
    std::fill(target_pos_.begin(), target_pos_.end(), params_.open_position);
    std::fill(latched_.begin(), latched_.end(), false);
    std::fill(contacted_.begin(), contacted_.end(), false);
    state_ = State::CLOSE;
    gated_close_ = false;
  }
  if (start_close_.exchange(false)) {
    auto msg_ptr = target_buf_.readFromRT();
    if (msg_ptr && *msg_ptr) {
      const auto & msg = **msg_ptr;
      for (std::size_t j = 0; j < params_.command_joints.size(); ++j) {
        for (std::size_t k = 0; k < msg.name.size() && k < msg.position.size(); ++k) {
          if (msg.name[k] == params_.command_joints[j]) {
            target_pos_[j] = msg.position[k];
            break;
          }
        }
      }
    }
    std::fill(latched_.begin(), latched_.end(), false);
    std::fill(contacted_.begin(), contacted_.end(), false);
    state_ = State::CLOSE;
    gated_close_ = true;
  }

  // 3. Command update.
  if (state_ == State::CLOSE) {
    // Thumb opposition drives to target ungated. Its progress gates the HOLD
    // transition together with the flexion latches, otherwise the short travel
    // flexion joints latch first and freeze opposition partway.
    bool opp_done = true;
    if (thumb_opp_slot_ >= 0) {
      const auto s = static_cast<std::size_t>(thumb_opp_slot_);
      if (std::abs(cmd_pos_[s] - target_pos_[s]) >= 1e-4) {
        cmd_pos_[s] = step_toward(cmd_pos_[s], target_pos_[s], params_.close_step);
        opp_done = false;
      }
    }
    // Each finger advances until contact (when gating) or until it reaches
    // its target.
    bool all_done = opp_done;
    for (std::size_t f = 0; f < n_fingers_; ++f) {
      const auto slot = static_cast<std::size_t>(flexion_slot_[f]);
      if (latched_[f]) {continue;}
      if (gated_close_ && contact_[f]) {contacted_[f] = true;}
      const bool reached = std::abs(cmd_pos_[slot] - target_pos_[slot]) < 1e-4;
      if (reached || (gated_close_ && contact_[f])) {
        // Frozen either way. Reaching the target with no contact ever seen
        // means the finger closed on air, which contacted_ records so the
        // diagnostic reports a miss, not a grip.
        latched_[f] = true;
      } else {
        cmd_pos_[slot] = step_toward(cmd_pos_[slot], target_pos_[slot], params_.close_step);
        all_done = false;
      }
    }
    if (all_done) {state_ = State::HOLD;}
  }
  // IDLE and HOLD leave cmd_pos_ unchanged.

  // 4. Write position commands.
  for (std::size_t j = 0; j < params_.command_joints.size(); ++j) {
    if (!command_interfaces_[cmd_idx_[j]].set_value(cmd_pos_[j])) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to set command on '%s'.", params_.command_joints[j].c_str());
    }
  }

  // 5. Diagnostics.
  publish_diagnostics(contact_);
  return return_type::OK;
}

void ContactGatedController::publish_diagnostics(const std::vector<bool> & contact)
{
  if (rt_contact_pub_ && rt_contact_pub_->trylock()) {
    auto & data = rt_contact_pub_->msg_.data;
    for (std::size_t f = 0; f < n_fingers_; ++f) {
      int code;
      if (params_.monitor_only) {
        code = contact[f] ? CONTACT_SENSING : CONTACT_NONE;
      } else if (latched_[f]) {
        code = contacted_[f] ? CONTACT_LATCHED : CONTACT_MISSED;
      } else {
        code = contact[f] ? CONTACT_SENSING : CONTACT_NONE;
      }
      data[f] = code;
    }
    rt_contact_pub_->unlockAndPublish();
  }
  if (rt_force_pub_ && rt_force_pub_->trylock()) {
    auto & data = rt_force_pub_->msg_.data;
    for (std::size_t f = 0; f < n_fingers_; ++f) {data[f] = finger_force_[f];}
    rt_force_pub_->unlockAndPublish();
  }
}

}  // namespace realhand_contact_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  realhand_contact_controller::ContactGatedController, controller_interface::ControllerInterface)
