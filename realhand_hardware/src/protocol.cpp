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

#include "realhand_hardware/protocol.hpp"

#include <algorithm>
#include <cmath>

namespace realhand_hardware::protocol
{

double raw_to_rad(double max_rad, std::uint8_t raw)
{
  return max_rad * (1.0 - static_cast<double>(raw) / 255.0);
}

std::uint8_t rad_to_raw(double max_rad, double rad)
{
  if (!(max_rad > 0.0)) {return 255;}
  const double clamped = std::clamp(rad, 0.0, max_rad);
  return static_cast<std::uint8_t>(std::lround(255.0 * (1.0 - clamped / max_rad)));
}

std::size_t encode_joint_frame(
  std::uint8_t frame_type, const std::uint8_t * values, std::size_t n_joints,
  std::uint8_t * payload)
{
  if (n_joints + 1 > MAX_PAYLOAD) {return 0;}
  payload[0] = frame_type;
  for (std::size_t i = 0; i < n_joints; ++i) {payload[1 + i] = values[i];}
  return n_joints + 1;
}

std::size_t encode_matrix_request(
  std::size_t finger, std::uint8_t mode_byte, std::uint8_t * payload)
{
  payload[0] = static_cast<std::uint8_t>(FRAME_MATRIX_BASE + finger);
  payload[1] = mode_byte;
  return 2;
}

bool decode_position_frame(
  const std::uint8_t * payload, std::size_t length, std::size_t n_joints,
  std::uint8_t * out)
{
  if (length != n_joints + 1 || payload[0] != FRAME_POSITION) {return false;}
  for (std::size_t i = 0; i < n_joints; ++i) {out[i] = payload[1 + i];}
  return true;
}

bool decode_matrix_row(
  const std::uint8_t * payload, std::size_t length, std::size_t n_fingers,
  std::size_t rows, std::size_t cols, MatrixRow & out)
{
  if (length < 2 + cols) {return false;}
  const std::uint8_t type = payload[0];
  if (type < FRAME_MATRIX_BASE || type >= FRAME_MATRIX_BASE + n_fingers) {return false;}
  const std::uint8_t row_key = payload[1];
  if (row_key % 16u != 0u) {return false;}
  const std::size_t row = row_key / 16u;
  if (row >= rows) {return false;}
  out.finger = type - FRAME_MATRIX_BASE;
  out.row = row;
  out.cols = payload + 2;
  return true;
}

}  // namespace realhand_hardware::protocol
