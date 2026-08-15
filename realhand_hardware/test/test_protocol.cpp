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

#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "realhand_hardware/hand_model.hpp"
#include "realhand_hardware/protocol.hpp"

using realhand_hardware::find_hand_model;
using realhand_hardware::HandModel;
namespace proto = realhand_hardware::protocol;

TEST(Protocol, RawToRadEndpoints)
{
  EXPECT_DOUBLE_EQ(proto::raw_to_rad(1.57, 0), 1.57);
  EXPECT_DOUBLE_EQ(proto::raw_to_rad(1.57, 255), 0.0);
  EXPECT_NEAR(proto::raw_to_rad(1.0, 127), 0.502, 1e-3);
}

TEST(Protocol, RadToRawClampsAndRoundTrips)
{
  EXPECT_EQ(proto::rad_to_raw(1.57, 0.0), 255);
  EXPECT_EQ(proto::rad_to_raw(1.57, 1.57), 0);
  EXPECT_EQ(proto::rad_to_raw(1.57, -1.0), 255);
  EXPECT_EQ(proto::rad_to_raw(1.57, 9.0), 0);
  for (int raw = 0; raw <= 255; ++raw) {
    const auto r = static_cast<std::uint8_t>(raw);
    EXPECT_EQ(proto::rad_to_raw(0.52, proto::raw_to_rad(0.52, r)), r);
  }
}

TEST(Protocol, EncodeJointFrame)
{
  std::array<std::uint8_t, 8> payload{};
  const std::uint8_t values[6] = {10, 20, 30, 40, 50, 60};
  ASSERT_EQ(proto::encode_joint_frame(proto::FRAME_POSITION, values, 6, payload.data()), 7u);
  EXPECT_EQ(payload[0], proto::FRAME_POSITION);
  EXPECT_EQ(payload[1], 10);
  EXPECT_EQ(payload[6], 60);
  const std::uint8_t too_many[8] = {};
  EXPECT_EQ(proto::encode_joint_frame(proto::FRAME_POSITION, too_many, 8, payload.data()), 0u);
}

TEST(Protocol, EncodeMatrixRequest)
{
  std::array<std::uint8_t, 8> payload{};
  ASSERT_EQ(proto::encode_matrix_request(3, 0xc6, payload.data()), 2u);
  EXPECT_EQ(payload[0], 0xb4);
  EXPECT_EQ(payload[1], 0xc6);
}

TEST(Protocol, DecodePositionFrameExactLength)
{
  const std::uint8_t good[7] = {0x01, 1, 2, 3, 4, 5, 6};
  std::uint8_t out[6] = {};
  ASSERT_TRUE(proto::decode_position_frame(good, 7, 6, out));
  EXPECT_EQ(out[0], 1);
  EXPECT_EQ(out[5], 6);

  // The tactile stream imposter, type 0x01 with eight bytes, must not decode.
  const std::uint8_t imposter[8] = {0x01, 0x10, 0, 0, 0, 0, 0, 0};
  EXPECT_FALSE(proto::decode_position_frame(imposter, 8, 6, out));

  // A read request echo, one byte, must not decode either.
  const std::uint8_t request[1] = {0x01};
  EXPECT_FALSE(proto::decode_position_frame(request, 1, 6, out));

  const std::uint8_t wrong_type[7] = {0x02, 1, 2, 3, 4, 5, 6};
  EXPECT_FALSE(proto::decode_position_frame(wrong_type, 7, 6, out));
}

TEST(Protocol, DecodeMatrixRow)
{
  const std::uint8_t row[8] = {0xb2, 0x30, 9, 8, 7, 6, 5, 4};
  proto::MatrixRow out{};
  ASSERT_TRUE(proto::decode_matrix_row(row, 8, 5, 12, 6, out));
  EXPECT_EQ(out.finger, 1u);
  EXPECT_EQ(out.row, 3u);
  EXPECT_EQ(out.cols[0], 9);
  EXPECT_EQ(out.cols[5], 4);

  const std::uint8_t bad_key[8] = {0xb2, 0x31, 9, 8, 7, 6, 5, 4};
  EXPECT_FALSE(proto::decode_matrix_row(bad_key, 8, 5, 12, 6, out));

  const std::uint8_t row_out_of_range[8] = {0xb2, 0xc0, 9, 8, 7, 6, 5, 4};
  EXPECT_FALSE(proto::decode_matrix_row(row_out_of_range, 8, 5, 12, 6, out));

  const std::uint8_t finger_out_of_range[8] = {0xb6, 0x00, 9, 8, 7, 6, 5, 4};
  EXPECT_FALSE(proto::decode_matrix_row(finger_out_of_range, 8, 5, 12, 6, out));

  const std::uint8_t short_frame[4] = {0xb1, 0x00, 1, 2};
  EXPECT_FALSE(proto::decode_matrix_row(short_frame, 4, 5, 12, 6, out));
}

TEST(HandModel, L6TableIsConsistent)
{
  const HandModel * l6 = find_hand_model("l6");
  ASSERT_NE(l6, nullptr);
  EXPECT_EQ(l6->name, "L6");
  EXPECT_EQ(l6->actuated.size(), 6u);
  EXPECT_EQ(l6->mimic.size(), 5u);
  EXPECT_EQ(l6->tactile_fingers.size(), 5u);
  EXPECT_EQ(l6->taxels_per_finger(), 72u);
  EXPECT_EQ(l6->can_id_right, 0x27u);
  EXPECT_EQ(l6->can_id_left, 0x28u);
  for (const auto & m : l6->mimic) {
    ASSERT_LT(m.parent_index, l6->actuated.size());
    EXPECT_GT(m.multiplier, 0.0);
  }
  EXPECT_EQ(l6->actuated_index("index_mcp_pitch"), 2);
  EXPECT_EQ(l6->actuated_index("nope"), -1);
  EXPECT_EQ(l6->mimic_index("thumb_ip"), 0);
  EXPECT_EQ(l6->mimic_index("index_mcp_pitch"), -1);
  EXPECT_EQ(find_hand_model("L99"), nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
