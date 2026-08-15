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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface_params.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realhand_contact_controller/contact_gated_controller.hpp"

using realhand_contact_controller::ContactGatedController;
using hardware_interface::CommandInterface;
using hardware_interface::StateInterface;
using CallbackReturn = controller_interface::CallbackReturn;

namespace
{

const std::vector<std::string> JOINTS = {
  "thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch", "middle_mcp_pitch", "ring_mcp_pitch",
  "pinky_mcp_pitch"};
const std::vector<std::string> FINGERS = {"thumb", "index", "middle", "ring", "pinky"};
constexpr double THRESHOLD = 300.0;
constexpr double STEP = 0.01;

// Drives the controller directly, no controller_manager, with loaned
// interfaces the test owns. The test sets tactile force on the state
// interfaces and reads what the controller wrote to the command interfaces.
class ContactGatedControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    for (const auto & j : JOINTS) {
      auto iface = std::make_shared<CommandInterface>(j, "position");
      EXPECT_TRUE(iface->set_value(0.0));
      commands_.push_back(iface);
    }
    for (const auto & f : FINGERS) {
      auto iface = std::make_shared<StateInterface>("tactile_" + f, "force");
      EXPECT_TRUE(iface->set_value(0.0));
      forces_.push_back(iface);
    }
  }

  void start_controller(bool monitor_only = false)
  {
    controller_ = std::make_unique<ContactGatedController>();
    controller_interface::ControllerInterfaceParams p;
    p.controller_name = "contact_gated_controller";
    p.update_rate = 100;
    p.controller_manager_update_rate = 100;
    p.node_options.parameter_overrides(
      {rclcpp::Parameter("contact_threshold", THRESHOLD), rclcpp::Parameter("close_step", STEP),
        rclcpp::Parameter("monitor_only", monitor_only)});
    ASSERT_EQ(controller_->init(p), controller_interface::return_type::OK);
    ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

    std::vector<hardware_interface::LoanedCommandInterface> cmd;
    std::vector<hardware_interface::LoanedStateInterface> st;
    if (!monitor_only) {
      for (auto & c : commands_) {cmd.emplace_back(c, nullptr);}
    }
    for (auto & s : forces_) {st.emplace_back(s, nullptr);}
    controller_->assign_interfaces(std::move(cmd), std::move(st));
    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);

    auto node = controller_->get_node();
    contact_sub_ = node->create_subscription<std_msgs::msg::Int32MultiArray>(
      "~/contact_state", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {last_contact_ = msg->data;});
    force_sub_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/finger_force", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {last_force_ = msg->data;});
    close_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
      "~/close_to", rclcpp::SystemDefaultsQoS());
    open_pub_ = node->create_publisher<std_msgs::msg::Bool>("~/open", rclcpp::SystemDefaultsQoS());
    executor_.add_node(node->get_node_base_interface());
    // Discovery between the intra process publisher and subscriber.
    spin_for(200);
  }

  void spin_for(int ms)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  void update(int cycles)
  {
    for (int i = 0; i < cycles; ++i) {
      ASSERT_EQ(
        controller_->update(controller_->get_node()->now(), rclcpp::Duration::from_seconds(0.01)),
        controller_interface::return_type::OK);
      executor_.spin_some();
    }
  }

  void request_close(double target)
  {
    sensor_msgs::msg::JointState msg;
    msg.name = JOINTS;
    msg.position.assign(JOINTS.size(), target);
    close_pub_->publish(msg);
    spin_for(50);
  }

  void request_open()
  {
    std_msgs::msg::Bool msg;
    msg.data = true;
    open_pub_->publish(msg);
    spin_for(50);
  }

  void set_force(std::size_t finger, double value)
  {
    ASSERT_TRUE(forces_[finger]->set_value(value));
  }

  double command(std::size_t joint) {return commands_[joint]->get_optional<double>().value();}

  // Wait until a diagnostic message from the current controller state has
  // been received, since the realtime publisher hands off to its own thread.
  std::vector<int> latest_contact()
  {
    last_contact_.clear();
    for (int i = 0; i < 100 && last_contact_.empty(); ++i) {
      update(1);
      spin_for(5);
    }
    return last_contact_;
  }

  std::unique_ptr<ContactGatedController> controller_;
  std::vector<std::shared_ptr<CommandInterface>> commands_;
  std::vector<std::shared_ptr<StateInterface>> forces_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr contact_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr force_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr close_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr open_pub_;
  std::vector<int> last_contact_;
  std::vector<double> last_force_;
};

}  // namespace

TEST_F(ContactGatedControllerTest, ClaimsExpectedInterfaces)
{
  start_controller();
  const auto cmd = controller_->command_interface_configuration();
  ASSERT_EQ(cmd.names.size(), JOINTS.size());
  EXPECT_EQ(cmd.names[0], "thumb_cmc_pitch/position");
  const auto st = controller_->state_interface_configuration();
  ASSERT_EQ(st.names.size(), FINGERS.size());
  EXPECT_EQ(st.names[4], "tactile_pinky/force");
}

TEST_F(ContactGatedControllerTest, IdleHoldsPositionAndReportsNoContact)
{
  start_controller();
  update(20);
  for (std::size_t j = 0; j < JOINTS.size(); ++j) {EXPECT_DOUBLE_EQ(command(j), 0.0);}
  const auto codes = latest_contact();
  ASSERT_EQ(codes.size(), FINGERS.size());
  for (int c : codes) {EXPECT_EQ(c, ContactGatedController::CONTACT_NONE);}
}

TEST_F(ContactGatedControllerTest, CloseStopsOnContactAndReportsGrip)
{
  start_controller();
  request_close(1.0);
  update(10);
  // Every joint has advanced 10 steps toward the target.
  for (std::size_t j = 0; j < JOINTS.size(); ++j) {EXPECT_NEAR(command(j), 10 * STEP, 1e-9);}

  // Index finger touches something.
  set_force(1, THRESHOLD);
  update(1);
  const double index_at_contact = command(2);
  update(30);
  // Index froze at contact, the others kept closing.
  EXPECT_DOUBLE_EQ(command(2), index_at_contact);
  EXPECT_NEAR(command(3), 41 * STEP, 1e-9);

  const auto codes = latest_contact();
  ASSERT_EQ(codes.size(), FINGERS.size());
  EXPECT_EQ(codes[1], ContactGatedController::CONTACT_LATCHED);
  EXPECT_EQ(codes[2], ContactGatedController::CONTACT_NONE);
}

TEST_F(ContactGatedControllerTest, ReachingTargetWithoutContactReportsMiss)
{
  start_controller();
  request_close(0.05);
  update(20);
  for (std::size_t j = 0; j < JOINTS.size(); ++j) {EXPECT_NEAR(command(j), 0.05, 1e-9);}
  const auto codes = latest_contact();
  ASSERT_EQ(codes.size(), FINGERS.size());
  for (int c : codes) {EXPECT_EQ(c, ContactGatedController::CONTACT_MISSED);}
}

TEST_F(ContactGatedControllerTest, ContactSensedBeforeCloseIsReportedAsSensing)
{
  start_controller();
  set_force(3, THRESHOLD + 1.0);
  const auto codes = latest_contact();
  ASSERT_EQ(codes.size(), FINGERS.size());
  EXPECT_EQ(codes[3], ContactGatedController::CONTACT_SENSING);
  // No close request, so nothing moved.
  for (std::size_t j = 0; j < JOINTS.size(); ++j) {EXPECT_DOUBLE_EQ(command(j), 0.0);}
}

TEST_F(ContactGatedControllerTest, OpenIsNotGatedByContact)
{
  start_controller();
  request_close(0.5);
  update(50);
  EXPECT_NEAR(command(2), 0.5, 1e-9);
  // Pads still pressed while opening must not stop the motion.
  for (std::size_t f = 0; f < FINGERS.size(); ++f) {set_force(f, THRESHOLD * 2);}
  request_open();
  update(60);
  for (std::size_t j = 0; j < JOINTS.size(); ++j) {EXPECT_NEAR(command(j), 0.0, 1e-9);}
  // After an ungated open the codes report plain contact, never latched or
  // missed.
  const auto codes = latest_contact();
  ASSERT_EQ(codes.size(), FINGERS.size());
  for (int c : codes) {EXPECT_EQ(c, ContactGatedController::CONTACT_SENSING);}
}

TEST_F(ContactGatedControllerTest, ThumbOppositionFinishesBeforeHold)
{
  start_controller();
  // Flexion joints get a short travel, opposition a long one.
  sensor_msgs::msg::JointState msg;
  msg.name = JOINTS;
  msg.position = {0.02, 0.5, 0.02, 0.02, 0.02, 0.02};
  close_pub_->publish(msg);
  spin_for(50);
  update(10);
  // Flexion joints latched at 0.02 after two steps, opposition keeps going.
  EXPECT_NEAR(command(0), 0.02, 1e-9);
  EXPECT_NEAR(command(1), 10 * STEP, 1e-9);
  update(45);
  EXPECT_NEAR(command(1), 0.5, 1e-9);
}

TEST_F(ContactGatedControllerTest, MonitorOnlyClaimsNoCommandsAndPublishes)
{
  start_controller(true);
  EXPECT_EQ(
    controller_->command_interface_configuration().type,
    controller_interface::interface_configuration_type::NONE);
  set_force(0, THRESHOLD);
  const auto codes = latest_contact();
  ASSERT_EQ(codes.size(), FINGERS.size());
  EXPECT_EQ(codes[0], ContactGatedController::CONTACT_SENSING);
  EXPECT_EQ(codes[1], ContactGatedController::CONTACT_NONE);
  spin_for(50);
  ASSERT_EQ(last_force_.size(), FINGERS.size());
  EXPECT_DOUBLE_EQ(last_force_[0], THRESHOLD);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
