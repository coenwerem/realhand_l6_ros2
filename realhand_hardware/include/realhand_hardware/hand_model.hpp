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

#ifndef REALHAND_HARDWARE__HAND_MODEL_HPP_
#define REALHAND_HARDWARE__HAND_MODEL_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace realhand_hardware
{

// One actuated joint as the hand firmware orders it inside the position
// frame. Byte 1 + index of the frame holds this joint. The firmware encodes
// each joint as a byte, 0 meaning fully closed at max_rad and 255 meaning
// fully open at zero radians.
struct ActuatedJoint
{
  std::string name;
  double max_rad;
};

// A passive joint the URDF drives from an actuated parent. The hand reports
// no state for it, so the driver synthesizes position as multiplier times
// the parent position.
struct MimicJoint
{
  std::string name;
  std::size_t parent_index;
  double multiplier;
};

// Everything the driver needs to know about one hand model. The CAN frame
// family (position 0x01, torque 0x02, speed 0x05, tactile matrices 0xb1 and
// up) is shared across the L6, L7, O6, and L10 per the vendor SDK. What
// changes per model is this table.
struct HandModel
{
  std::string name;
  std::vector<ActuatedJoint> actuated;
  std::vector<MimicJoint> mimic;
  // Finger pads with a taxel matrix, in the order the firmware numbers them
  // starting at frame type 0xb1.
  std::vector<std::string> tactile_fingers;
  std::size_t taxel_rows;
  std::size_t taxel_cols;
  // Parameter byte sent with each matrix request. 0xc6 on the L6, 0xa4 on
  // the O6 per the vendor SDK.
  std::uint8_t matrix_mode_byte;
  std::uint32_t can_id_right;
  std::uint32_t can_id_left;

  std::size_t taxels_per_finger() const {return taxel_rows * taxel_cols;}
  // Index into actuated for a bare joint name, or -1.
  int actuated_index(const std::string & bare_name) const;
  // Index into mimic for a bare joint name, or -1.
  int mimic_index(const std::string & bare_name) const;
};

// Look a model up by name, case insensitive. Returns nullptr when the model
// is unknown. Only models validated on hardware ship in the table.
const HandModel * find_hand_model(const std::string & name);

// Names of every model in the table, for error messages.
std::vector<std::string> hand_model_names();

}  // namespace realhand_hardware

#endif  // REALHAND_HARDWARE__HAND_MODEL_HPP_
