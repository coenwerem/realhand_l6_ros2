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

#include "realhand_hardware/realhand_system.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "realhand_hardware/protocol.hpp"

namespace realhand_hardware
{

namespace proto = protocol;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{

bool parse_bool(const std::string & v)
{
  return v == "true" || v == "True" || v == "1" || v == "yes";
}

}  // namespace

// ---------------------------------------------------------------- lifecycle

CallbackReturn RealHandSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params_in)
{
  if (SystemInterface::on_init(params_in) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  const auto & params = info_.hardware_parameters;
  auto param = [&params](const char * key, const std::string & fallback) {
      auto it = params.find(key);
      return it == params.end() ? fallback : it->second;
    };

  const std::string model_name = param("model", "L6");
  model_ = find_hand_model(model_name);
  if (model_ == nullptr) {
    std::string known;
    for (const auto & n : hand_model_names()) {known += n + " ";}
    RCLCPP_FATAL(
      get_logger(), "Unknown hand model '%s'. Known models are %s", model_name.c_str(),
      known.c_str());
    return CallbackReturn::ERROR;
  }

  can_interface_ = param("can_interface", "can0");
  hand_side_ = param("hand_side", "right");
  if (hand_side_ != "right" && hand_side_ != "left") {
    RCLCPP_FATAL(get_logger(), "hand_side must be right or left, got '%s'.", hand_side_.c_str());
    return CallbackReturn::ERROR;
  }
  can_id_ = hand_side_ == "left" ? model_->can_id_left : model_->can_id_right;
  if (params.count("can_id")) {
    can_id_ = static_cast<std::uint32_t>(std::stoul(params.at("can_id"), nullptr, 0));
  }
  joint_prefix_ = param("joint_prefix", hand_side_ + "_");
  enable_tactile_ = parse_bool(param("enable_tactile", "true"));
  tactile_period_ms_ = std::stoi(param("tactile_period_ms", "50"));
  position_request_decimation_ =
    static_cast<std::uint32_t>(std::max(1, std::stoi(param("position_request_decimation", "4"))));
  activation_speed_ =
    static_cast<std::uint8_t>(std::clamp(std::stoi(param("activation_speed", "80")), 0, 255));
  activation_torque_ =
    static_cast<std::uint8_t>(std::clamp(std::stoi(param("activation_torque", "200")), 0, 255));
  taxel_topic_ = param("taxel_topic", "");

  const std::size_t n_act = model_->actuated.size();
  raw_position_.assign(n_act, 0);
  raw_matrix_.assign(model_->tactile_fingers.size() * model_->taxels_per_finger(), 0);
  position_snapshot_.assign(n_act, 0);
  matrix_snapshot_.assign(raw_matrix_.size(), 0);
  command_raw_.assign(n_act, 0);
  last_command_raw_.assign(n_act, 0);

  // Map every URDF joint onto the model table and validate its interfaces.
  joint_slots_.clear();
  for (const auto & joint : info_.joints) {
    const std::string bare = strip_prefix(joint.name);
    const int act = model_->actuated_index(bare);
    const int mim = model_->mimic_index(bare);
    if (act >= 0) {
      if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          get_logger(), "Actuated joint '%s' needs one position command interface.",
          joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      joint_slots_.push_back({true, static_cast<std::size_t>(act)});
    } else if (mim >= 0) {
      if (!joint.command_interfaces.empty()) {
        RCLCPP_FATAL(
          get_logger(), "Mimic joint '%s' must not declare command interfaces.",
          joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      joint_slots_.push_back({false, static_cast<std::size_t>(mim)});
    } else {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' is not a %s joint (prefix '%s' stripped, looked up '%s').",
        joint.name.c_str(), model_->name.c_str(), joint_prefix_.c_str(), bare.c_str());
      return CallbackReturn::ERROR;
    }
    for (const auto & si : joint.state_interfaces) {
      if (si.name != hardware_interface::HW_IF_POSITION &&
        si.name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' declares unsupported state interface '%s'.",
          joint.name.c_str(), si.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
  }

  RCLCPP_INFO(
    get_logger(), "%s %s hand on %s, CAN id 0x%02X, %zu joints mapped, tactile %s.",
    model_->name.c_str(), hand_side_.c_str(), can_interface_.c_str(), can_id_,
    joint_slots_.size(), enable_tactile_ ? "on" : "off");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RealHandSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!resolve_handles()) {return CallbackReturn::ERROR;}
  if (!can_open()) {return CallbackReturn::ERROR;}

  if (!taxel_topic_.empty()) {
    try {
      taxel_node_ = std::make_shared<rclcpp::Node>("realhand_taxel_publisher");
      taxel_pub_ = taxel_node_->create_publisher<std_msgs::msg::String>(
        taxel_topic_, rclcpp::SystemDefaultsQoS());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Taxel publisher unavailable (%s).", e.what());
      taxel_pub_.reset();
      taxel_node_.reset();
    }
  }

  position_seen_.store(false);
  recv_running_.store(true);
  recv_thread_ = std::thread(&RealHandSystem::receive_loop, this);

  if (!read_initial_position()) {
    RCLCPP_WARN(
      get_logger(),
      "No position reply from the hand. Either the hand is silent on %s or every joint "
      "reads fully open. Continuing with the last known raw state.",
      can_interface_.c_str());
  }
  safe_init_sequence();

  // Seed state and command with the physical position so the first write()
  // holds the hand where it is until a controller commands otherwise.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    position_snapshot_ = raw_position_;
  }
  for (std::size_t i = 0; i < joint_slots_.size(); ++i) {
    const auto & slot = joint_slots_[i];
    double rad;
    if (slot.actuated) {
      rad = proto::raw_to_rad(model_->actuated[slot.index].max_rad, position_snapshot_[slot.index]);
      std::ignore = set_command(position_command_[i], rad, false);
    } else {
      const auto & m = model_->mimic[slot.index];
      rad = m.multiplier *
        proto::raw_to_rad(model_->actuated[m.parent_index].max_rad, position_snapshot_[m.parent_index]);
    }
    std::ignore = set_state(position_state_[i], rad, false);
    if (velocity_state_[i]) {std::ignore = set_state(velocity_state_[i], 0.0, false);}
  }
  last_command_raw_ = position_snapshot_;

  RCLCPP_INFO(get_logger(), "%s hand active.", model_->name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn RealHandSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  recv_running_.store(false);
  if (recv_thread_.joinable()) {recv_thread_.join();}
  taxel_pub_.reset();
  taxel_node_.reset();
  can_close();
  RCLCPP_INFO(get_logger(), "%s hand inactive.", model_->name.c_str());
  return CallbackReturn::SUCCESS;
}

// ------------------------------------------------------------- read / write

return_type RealHandSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Ask for position every N cycles. Every request preempts the in flight
  // tactile stream, so once per cycle starved the matrices.
  if (++position_request_counter_ % position_request_decimation_ == 0) {
    const std::uint8_t request[1] = {proto::FRAME_POSITION};
    can_send(request, 1);
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    position_snapshot_ = raw_position_;
    matrix_snapshot_ = raw_matrix_;
  }

  for (std::size_t i = 0; i < joint_slots_.size(); ++i) {
    const auto & slot = joint_slots_[i];
    double rad;
    if (slot.actuated) {
      rad = proto::raw_to_rad(model_->actuated[slot.index].max_rad, position_snapshot_[slot.index]);
    } else {
      const auto & m = model_->mimic[slot.index];
      rad = m.multiplier *
        proto::raw_to_rad(model_->actuated[m.parent_index].max_rad, position_snapshot_[m.parent_index]);
    }
    std::ignore = set_state(position_state_[i], rad, false);
    if (velocity_state_[i]) {std::ignore = set_state(velocity_state_[i], 0.0, false);}
  }

  // One summed force per pad. Five scalars keep the consuming controller's
  // per cycle interface reads cheap where 360 taxel interfaces overran the
  // control loop.
  const std::size_t per_finger = model_->taxels_per_finger();
  for (std::size_t f = 0; f < force_state_.size(); ++f) {
    if (!force_state_[f]) {continue;}
    long sum = 0;
    const std::uint8_t * pad = matrix_snapshot_.data() + f * per_finger;
    for (std::size_t t = 0; t < per_finger; ++t) {sum += pad[t];}
    std::ignore = set_state(force_state_[f], static_cast<double>(sum), false);
  }
  return return_type::OK;
}

return_type RealHandSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // Joints nobody commands hold their last reported position.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    command_raw_ = raw_position_;
  }
  for (std::size_t i = 0; i < joint_slots_.size(); ++i) {
    const auto & slot = joint_slots_[i];
    if (!slot.actuated) {continue;}
    double cmd = std::numeric_limits<double>::quiet_NaN();
    if (get_command(position_command_[i], cmd, false) && std::isfinite(cmd)) {
      command_raw_[slot.index] = proto::rad_to_raw(model_->actuated[slot.index].max_rad, cmd);
    }
  }

  // Send only on change. A held position resent every cycle floods the bus
  // and preempts the tactile stream.
  if (command_raw_ != last_command_raw_) {
    std::uint8_t payload[proto::MAX_PAYLOAD];
    const std::size_t len = proto::encode_joint_frame(
      proto::FRAME_POSITION, command_raw_.data(), command_raw_.size(), payload);
    if (len > 0 && can_send(payload, len)) {
      last_command_raw_ = command_raw_;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Position command not sent on %s.",
        can_interface_.c_str());
    }
  }
  return return_type::OK;
}

// ------------------------------------------------------------------ helpers

std::string RealHandSystem::strip_prefix(const std::string & name) const
{
  if (!joint_prefix_.empty() && name.size() > joint_prefix_.size() &&
    name.compare(0, joint_prefix_.size(), joint_prefix_) == 0)
  {
    return name.substr(joint_prefix_.size());
  }
  return name;
}

bool RealHandSystem::resolve_handles()
{
  const std::size_t n = joint_slots_.size();
  position_state_.assign(n, nullptr);
  velocity_state_.assign(n, nullptr);
  position_command_.assign(n, nullptr);
  for (std::size_t i = 0; i < n; ++i) {
    const std::string & jn = info_.joints[i].name;
    const std::string pos = jn + "/" + hardware_interface::HW_IF_POSITION;
    const std::string vel = jn + "/" + hardware_interface::HW_IF_VELOCITY;
    if (!has_state(pos)) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' declares no position state interface.", jn.c_str());
      return false;
    }
    position_state_[i] = get_state_interface_handle(pos);
    if (has_state(vel)) {velocity_state_[i] = get_state_interface_handle(vel);}
    if (joint_slots_[i].actuated) {position_command_[i] = get_command_interface_handle(pos);}
  }

  // Tactile sensors are optional in the XML. A finger without a sensor is
  // still decoded, just not exported.
  force_state_.assign(model_->tactile_fingers.size(), nullptr);
  std::size_t exported = 0;
  for (std::size_t f = 0; f < model_->tactile_fingers.size(); ++f) {
    const std::string bare = "tactile_" + model_->tactile_fingers[f] + "/force";
    const std::string prefixed = joint_prefix_ + bare;
    if (has_state(prefixed)) {
      force_state_[f] = get_state_interface_handle(prefixed);
    } else if (has_state(bare)) {
      force_state_[f] = get_state_interface_handle(bare);
    }
    if (force_state_[f]) {++exported;}
  }
  if (enable_tactile_ && exported == 0) {
    RCLCPP_WARN(
      get_logger(),
      "Tactile enabled but the XML declares no tactile_<finger> sensor with a force "
      "state interface, so no pad force is exported.");
  }
  return true;
}

bool RealHandSystem::read_initial_position()
{
  using namespace std::chrono_literals;
  for (int attempt = 0; attempt < 5 && recv_running_.load(); ++attempt) {
    const std::uint8_t request[1] = {proto::FRAME_POSITION};
    can_send(request, 1);
    std::this_thread::sleep_for(30ms);
    if (position_seen_.load()) {return true;}
  }
  return false;
}

void RealHandSystem::safe_init_sequence()
{
  using namespace std::chrono_literals;
  const std::size_t n = model_->actuated.size();
  std::uint8_t payload[proto::MAX_PAYLOAD];
  std::vector<std::uint8_t> values(n, 0);

  // Command the current position before enabling torque, so the servos do
  // not snap to a stale target when torque comes on.
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    values = raw_position_;
  }
  std::size_t len = proto::encode_joint_frame(proto::FRAME_POSITION, values.data(), n, payload);
  can_send(payload, len);
  std::this_thread::sleep_for(10ms);
  can_send(payload, len);
  std::this_thread::sleep_for(10ms);

  std::fill(values.begin(), values.end(), activation_speed_);
  len = proto::encode_joint_frame(proto::FRAME_SPEED, values.data(), n, payload);
  can_send(payload, len);
  std::this_thread::sleep_for(3ms);
  can_send(payload, len);
  std::this_thread::sleep_for(50ms);

  std::fill(values.begin(), values.end(), activation_torque_);
  len = proto::encode_joint_frame(proto::FRAME_TORQUE, values.data(), n, payload);
  can_send(payload, len);
  std::this_thread::sleep_for(50ms);

  const std::uint8_t request[1] = {proto::FRAME_POSITION};
  can_send(request, 1);
  std::this_thread::sleep_for(20ms);
}

// ---------------------------------------------------------------------- CAN

bool RealHandSystem::can_open()
{
  can_socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_socket_ < 0) {
    RCLCPP_FATAL(get_logger(), "CAN socket failed, %s.", std::strerror(errno));
    return false;
  }
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
    RCLCPP_FATAL(
      get_logger(), "CAN interface '%s' not found, %s.", can_interface_.c_str(),
      std::strerror(errno));
    can_close();
    return false;
  }
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(can_socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_FATAL(get_logger(), "CAN bind failed, %s.", std::strerror(errno));
    can_close();
    return false;
  }
  struct can_filter filters[2];
  filters[0].can_id = can_id_;
  filters[0].can_mask = CAN_SFF_MASK;
  filters[1].can_id = can_id_ + proto::REPLY_ID_OFFSET;
  filters[1].can_mask = CAN_SFF_MASK;
  ::setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filters, sizeof(filters));
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 10000;
  ::setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  return true;
}

void RealHandSystem::can_close()
{
  if (can_socket_ >= 0) {
    ::close(can_socket_);
    can_socket_ = -1;
  }
}

bool RealHandSystem::can_send(const std::uint8_t * payload, std::size_t length)
{
  if (can_socket_ < 0 || length > proto::MAX_PAYLOAD) {return false;}
  std::lock_guard<std::mutex> lock(tx_mutex_);
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id_;
  frame.can_dlc = static_cast<std::uint8_t>(length);
  std::memcpy(frame.data, payload, length);
  return ::write(can_socket_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}

void RealHandSystem::receive_loop()
{
  using clock = std::chrono::steady_clock;
  const std::size_t n_act = model_->actuated.size();
  const std::size_t n_fingers = model_->tactile_fingers.size();
  const std::size_t rows = model_->taxel_rows;
  const std::size_t cols = model_->taxel_cols;
  auto last_request = clock::now();
  std::vector<std::uint8_t> position(n_act, 0);

  while (recv_running_.load()) {
    // Matrix requests go out from this thread as a burst for every pad.
    // Interleaving them with the control cycle disrupted the hand's row
    // streaming.
    const auto now = clock::now();
    if (enable_tactile_ && now - last_request >= std::chrono::milliseconds(tactile_period_ms_)) {
      last_request = now;
      std::uint8_t payload[proto::MAX_PAYLOAD];
      for (std::size_t f = 0; f < n_fingers; ++f) {
        const std::size_t len = proto::encode_matrix_request(f, model_->matrix_mode_byte, payload);
        can_send(payload, len);
      }
      if (taxel_pub_) {publish_taxel_matrices();}
    }

    struct can_frame frame;
    const ssize_t nbytes = ::read(can_socket_, &frame, sizeof(frame));
    if (nbytes <= 0) {continue;}
    const std::uint32_t id = frame.can_id & CAN_SFF_MASK;
    if (id != can_id_ && id != can_id_ + proto::REPLY_ID_OFFSET) {continue;}
    if (frame.can_dlc < 1) {continue;}

    if (proto::decode_position_frame(frame.data, frame.can_dlc, n_act, position.data())) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      raw_position_ = position;
      position_seen_.store(true);
      continue;
    }
    proto::MatrixRow row{};
    if (proto::decode_matrix_row(frame.data, frame.can_dlc, n_fingers, rows, cols, row)) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      std::uint8_t * dst = raw_matrix_.data() + row.finger * rows * cols + row.row * cols;
      std::memcpy(dst, row.cols, cols);
    }
  }
}

void RealHandSystem::publish_taxel_matrices()
{
  // JSON with one "<finger>_matrix" key per pad, each a rows x cols list, plus
  // a stamp. Same shape the vendor SDK publishes, so existing recorders parse it.
  std::vector<std::uint8_t> snap;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    snap = raw_matrix_;
  }
  const std::size_t rows = model_->taxel_rows;
  const std::size_t cols = model_->taxel_cols;
  std::ostringstream os;
  os << "{";
  for (std::size_t f = 0; f < model_->tactile_fingers.size(); ++f) {
    os << "\"" << model_->tactile_fingers[f] << "_matrix\":[";
    for (std::size_t r = 0; r < rows; ++r) {
      os << "[";
      for (std::size_t c = 0; c < cols; ++c) {
        os << static_cast<int>(snap[(f * rows + r) * cols + c]);
        if (c + 1 < cols) {os << ",";}
      }
      os << (r + 1 < rows ? "]," : "]");
    }
    os << "],";
  }
  const std::int64_t ns = taxel_node_->now().nanoseconds();
  os << "\"stamp\":{\"secs\":" << (ns / 1000000000LL) << ",\"nsecs\":" << (ns % 1000000000LL)
     << "}}";
  std_msgs::msg::String msg;
  msg.data = os.str();
  taxel_pub_->publish(msg);
}

}  // namespace realhand_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(realhand_hardware::RealHandSystem, hardware_interface::SystemInterface)
