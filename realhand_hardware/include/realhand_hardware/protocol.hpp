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

#ifndef REALHAND_HARDWARE__PROTOCOL_HPP_
#define REALHAND_HARDWARE__PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

// Pure functions over the RealHand CAN frame family. No sockets, no ROS, so
// the codec is unit testable and the SystemInterface stays thin.
//
// Frame layout, all standard 11 bit identifiers. Byte 0 of the payload is
// the frame type, the rest is type specific.
//   0x01  position, byte 1 + i is joint i (0 closed, 255 open). A one byte
//         frame is a read request, the hand answers with the full frame.
//   0x02  torque, byte 1 + i is joint i (0 to 255).
//   0x05  speed, byte 1 + i is joint i (0 to 255).
//   0xb1  tactile matrix request or row for finger 0, 0xb2 finger 1, and so
//         on. A request is [type, mode_byte]. A row reply is
//         [type, row_key, c0 .. c(cols-1)] with row_key = 16 * row.
namespace realhand_hardware::protocol
{

constexpr std::uint8_t FRAME_POSITION = 0x01;
constexpr std::uint8_t FRAME_TORQUE = 0x02;
constexpr std::uint8_t FRAME_SPEED = 0x05;
constexpr std::uint8_t FRAME_MATRIX_BASE = 0xb1;

// The hand answers on its own identifier and on identifier + 8.
constexpr std::uint32_t REPLY_ID_OFFSET = 8;

constexpr std::size_t MAX_PAYLOAD = 8;

// Byte to radians. raw 0 maps onto max_rad, raw 255 onto zero.
double raw_to_rad(double max_rad, std::uint8_t raw);

// Radians to byte, clamped into [0, max_rad] first.
std::uint8_t rad_to_raw(double max_rad, double rad);

// Fill payload with a position, torque, or speed frame for n_joints joints.
// Returns the payload length, or 0 when n_joints does not fit in one frame.
std::size_t encode_joint_frame(
  std::uint8_t frame_type, const std::uint8_t * values, std::size_t n_joints,
  std::uint8_t * payload);

// Fill payload with a matrix request for one finger. Returns the length, 2.
std::size_t encode_matrix_request(
  std::size_t finger, std::uint8_t mode_byte, std::uint8_t * payload);

// True when payload is a full position reply for n_joints joints, in which
// case the joint bytes are copied into out. The length test is exact on
// purpose. During tactile streaming the L6 also emits a type 0x01 frame of
// length 8 whose remaining bytes are zero, and accepting it clenched the
// model to fully closed.
bool decode_position_frame(
  const std::uint8_t * payload, std::size_t length, std::size_t n_joints,
  std::uint8_t * out);

struct MatrixRow
{
  std::size_t finger;
  std::size_t row;
  const std::uint8_t * cols;
};

// True when payload is one tactile matrix row for a hand with n_fingers pads
// of rows x cols taxels. Rows whose key is not a multiple of 16 or that fall
// outside the pad are rejected.
bool decode_matrix_row(
  const std::uint8_t * payload, std::size_t length, std::size_t n_fingers,
  std::size_t rows, std::size_t cols, MatrixRow & out);

}  // namespace realhand_hardware::protocol

#endif  // REALHAND_HARDWARE__PROTOCOL_HPP_
