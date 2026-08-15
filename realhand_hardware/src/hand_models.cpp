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

#include "realhand_hardware/hand_model.hpp"

#include <algorithm>
#include <cctype>

namespace realhand_hardware
{

namespace
{

std::string to_upper(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::toupper(c));});
  return s;
}

// RealHand L6. Six actuated joints, five coupled distal joints, five 12x6
// taxel pads. max_rad values match the URDF upper limits so that raw 0 maps
// onto the URDF upper bound and every mimic product stays inside the mimic
// joint's own limit (thumb_ip 0.52 * 1.83 = 0.95 under 0.96, the four dip
// joints 1.57 * 0.89 = 1.40 under 1.4). Verified on a right L6 unit over CAN.
const HandModel L6{
  "L6",
  {
    {"thumb_cmc_pitch", 0.52},
    {"thumb_cmc_yaw", 1.54},
    {"index_mcp_pitch", 1.57},
    {"middle_mcp_pitch", 1.57},
    {"ring_mcp_pitch", 1.57},
    {"pinky_mcp_pitch", 1.57},
  },
  {
    {"thumb_ip", 0, 1.83},
    {"index_dip", 2, 0.89},
    {"middle_dip", 3, 0.89},
    {"ring_dip", 4, 0.89},
    {"pinky_dip", 5, 0.89},
  },
  {"thumb", "index", "middle", "ring", "pinky"},
  12,
  6,
  0xc6,
  0x27,
  0x28,
};

const HandModel * const MODELS[] = {&L6};

}  // namespace

int HandModel::actuated_index(const std::string & bare_name) const
{
  for (std::size_t i = 0; i < actuated.size(); ++i) {
    if (actuated[i].name == bare_name) {return static_cast<int>(i);}
  }
  return -1;
}

int HandModel::mimic_index(const std::string & bare_name) const
{
  for (std::size_t i = 0; i < mimic.size(); ++i) {
    if (mimic[i].name == bare_name) {return static_cast<int>(i);}
  }
  return -1;
}

const HandModel * find_hand_model(const std::string & name)
{
  const std::string wanted = to_upper(name);
  for (const HandModel * m : MODELS) {
    if (m->name == wanted) {return m;}
  }
  return nullptr;
}

std::vector<std::string> hand_model_names()
{
  std::vector<std::string> out;
  for (const HandModel * m : MODELS) {out.push_back(m->name);}
  return out;
}

}  // namespace realhand_hardware
