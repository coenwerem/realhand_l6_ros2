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

#ifndef REALHAND_HARDWARE__REALHAND_SYSTEM_HPP_
#define REALHAND_HARDWARE__REALHAND_SYSTEM_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"

#include "realhand_hardware/hand_model.hpp"

namespace realhand_hardware
{

// ros2_control SystemInterface for RealHand dexterous hands over Linux
// SocketCAN. One instance drives one hand.
//
// Hardware parameters, all optional.
//   model                        hand model, default L6
//   can_interface                SocketCAN device, default can0
//   hand_side                    right or left, selects the default CAN id
//   can_id                       explicit CAN id, overrides hand_side
//   joint_prefix                 prefix stripped from URDF joint and sensor
//                                names before lookup, default hand_side + "_"
//                                and the bare name is always accepted too
//   enable_tactile               request taxel matrices, default true
//   tactile_period_ms            matrix request burst period, default 50
//   position_request_decimation  request position every N cycles, default 4
//   activation_speed             speed byte sent on activation, default 80
//   activation_torque            torque byte sent on activation, default 200
//   taxel_topic                  when set, publish the raw taxel grids as
//                                JSON on this topic from the receiver thread
//
// Exported interfaces come from the ros2_control XML. Each actuated joint
// takes a position command and reports position. An actuated joint may also
// declare speed and torque command interfaces, raw 0 to 255 setpoints the
// hand applies to that joint, sent as a frame whenever any value changes and
// seeded from activation_speed and activation_torque so a controller reads a
// defined value. Mimic joints report position only. A velocity state
// interface is filled with zero when declared. Each tactile finger reports
// the summed pad force through a sensor named tactile_<finger> with a force
// state interface.
class RealHandSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RealHandSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Maps one entry of info_.joints onto the model table.
  struct JointSlot
  {
    bool actuated;
    std::size_t index;
  };

  bool can_open();
  void can_close();
  bool can_send(const std::uint8_t * payload, std::size_t length);
  void receive_loop();
  void publish_taxel_matrices();
  std::string strip_prefix(const std::string & name) const;
  bool resolve_handles();
  bool read_initial_position();
  void safe_init_sequence();
  // Send one setpoint frame (speed or torque) when the values differ from
  // the last frame sent. Returns true when a frame went out or none was due.
  bool send_setpoints_on_change(
    std::uint8_t frame_type, const std::vector<std::uint8_t> & values,
    std::vector<std::uint8_t> & last_sent);

  const HandModel * model_{nullptr};
  std::string can_interface_{"can0"};
  std::uint32_t can_id_{0};
  std::string hand_side_{"right"};
  std::string joint_prefix_;
  bool enable_tactile_{true};
  int tactile_period_ms_{50};
  std::uint32_t position_request_decimation_{4};
  std::uint8_t activation_speed_{80};
  std::uint8_t activation_torque_{200};
  std::string taxel_topic_;

  int can_socket_{-1};
  // One transmitter at a time. The control thread sends position, the
  // receiver thread sends matrix requests, and the USB adapter corrupted
  // frames when both hit it together.
  std::mutex tx_mutex_;

  std::thread recv_thread_;
  std::atomic<bool> recv_running_{false};

  // Raw hand state in CAN order, written by the receiver thread and copied
  // under state_mutex_ by read(), write(), and on_activate().
  std::mutex state_mutex_;
  std::vector<std::uint8_t> raw_position_;
  std::vector<std::uint8_t> raw_matrix_;   // finger major, then row major
  std::atomic<bool> position_seen_{false};

  std::vector<JointSlot> joint_slots_;
  std::vector<hardware_interface::StateInterface::SharedPtr> position_state_;
  std::vector<hardware_interface::StateInterface::SharedPtr> velocity_state_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> position_command_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> speed_command_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> torque_command_;
  std::vector<hardware_interface::StateInterface::SharedPtr> force_state_;

  // Scratch buffers sized once so read() and write() do not allocate.
  std::vector<std::uint8_t> position_snapshot_;
  std::vector<std::uint8_t> matrix_snapshot_;
  std::vector<std::uint8_t> command_raw_;
  std::vector<std::uint8_t> last_command_raw_;
  std::vector<std::uint8_t> speed_raw_;
  std::vector<std::uint8_t> last_speed_raw_;
  std::vector<std::uint8_t> torque_raw_;
  std::vector<std::uint8_t> last_torque_raw_;
  std::uint32_t position_request_counter_{0};

  rclcpp::Node::SharedPtr taxel_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr taxel_pub_;
};

}  // namespace realhand_hardware

#endif  // REALHAND_HARDWARE__REALHAND_SYSTEM_HPP_
