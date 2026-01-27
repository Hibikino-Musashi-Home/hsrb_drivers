/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <hsrb_power_ecu/i_network.hpp>
#include <hsrb_power_ecu/serial_network.hpp>

#include "../src/battery_diagnostic_task.hpp"
#include "../src/battery_state_publisher.hpp"
#include "../src/bool_state_publisher.hpp"
#include "../src/get_parameter.hpp"
#include "../src/led_command_subscriber.hpp"
#include "../src/power_ecu_protocol.hpp"
#include "../src/runstop_state_publisher.hpp"

#include "network_mock.hpp"


namespace {
// ECU parameter names and topic names to be published
const std::vector<std::pair<std::string, std::string>> kParamAndTopicNames {
  {"is_bumper_bumper1", "base_f_bumper_sensor"}, {"is_bumper_bumper2", "base_b_bumper_sensor"},
  {"is_powerecu_sw_kinoko", "emergency_stop_button"}, {"is_powerecu_sw_w_stop", "wireless_stop_button"},
  {"is_powerecu_sw_w_sel", "wireless_stop_enable"}, {"is_powerecu_bat_stat", "battery_charging"},
  {"is_bumper_prox1", "base_magnetic_sensor_1"}, {"is_bumper_prox2", "base_magnetic_sensor_2"}
};

const rclcpp::Duration kLoopTimeout = rclcpp::Duration::from_seconds(1.0);

class IPublishChecker {
 public:
  IPublishChecker() : is_update_(false), count_(0) {}
  virtual ~IPublishChecker() = default;
  void Reset() {is_update_ = false; count_ = 0;}
  bool CheckUpdated() const {return is_update_;}
  uint32_t GetCount() const {return count_;}

 protected:
  bool is_update_;
  void IncrementCount() {++count_;}

 private:
  uint32_t count_;
};

class BatteryStatePublishChecker : public IPublishChecker {
 public:
  BatteryStatePublishChecker(const rclcpp::Node::SharedPtr& node,
                             const std::string& topic_name) {
    subscriber_ = node->create_subscription<sensor_msgs::msg::BatteryState>(topic_name, rclcpp::QoS(1),
        std::bind(&BatteryStatePublishChecker::Callback, this, std::placeholders::_1));
  }

  sensor_msgs::msg::BatteryState GetValue() const {return last_battery_state_;}

 private:
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscriber_;
  sensor_msgs::msg::BatteryState last_battery_state_;

  void Callback(const sensor_msgs::msg::BatteryState::SharedPtr message) {
    IncrementCount();
    last_battery_state_ = *message;
    is_update_ = true;
  }
};

class BoolPublishChecker : public IPublishChecker {
 public:
  BoolPublishChecker(const rclcpp::Node::SharedPtr& node,
                     const std::string& topic_name) {
    subscriber_ = node->create_subscription<std_msgs::msg::Bool>(topic_name, rclcpp::QoS(1),
        std::bind(&BoolPublishChecker::Callback, this, std::placeholders::_1));
  }

  bool CheckExpected(const bool expected_value) const {return last_value_ == expected_value;}

 private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
  bool last_value_;

  void Callback(const std_msgs::msg::Bool::SharedPtr message) {
    IncrementCount();
    last_value_ = message->data;
    is_update_ = true;
  }
};

class DiagnosticsChecker : public IPublishChecker {
 public:
  DiagnosticsChecker(const rclcpp::Node::SharedPtr& node, const std::string& diagnostics_name)
      : diagnostics_name_(diagnostics_name) {
    subscriber_ = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1),
        std::bind(&DiagnosticsChecker::Callback, this, std::placeholders::_1));
  }

  uint8_t GetLevel() const { return last_status_.level; }
  std::string GetMessage() const { return last_status_.message; }
  std::string GetHardwareId() const { return last_status_.hardware_id; }
  std::string GetValue(const std::string& key) const {
    for (const auto& value : last_status_.values) {
      if (value.key == key) {
        return value.value;
      }
    }
    return "";
  }

 private:
  std::string diagnostics_name_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscriber_;
  diagnostic_msgs::msg::DiagnosticStatus last_status_;

  void Callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr message) {
    IncrementCount();
    for (const auto& status : message->status) {
      if ((status.name.find(diagnostics_name_) != std::string::npos) && (status.message != "Node starting up")) {
        last_status_ = status;
        is_update_ = true;
        return;
      }
    }
  }
};

}  // anonymous namespace

namespace hsrb_power_ecu {
class MainLoopTest : public ::testing::Test {
 protected:
  void SetUp() override {
    test_node_ = rclcpp::Node::make_shared("test_node");
    loop_node_ = rclcpp::Node::make_shared("loop_node");
    loop_node_->declare_parameter<double>("runstop_button/publish_rate", 100.0);
    loop_node_->declare_parameter<double>("battery_state/publish_rate", 100.0);
    for (const auto& name : kParamAndTopicNames) {
      loop_node_->declare_parameter<double>(name.second + "/publish_rate", 100.0);
    }
    PrepareMainLoop();
  }

  void TearDown() override {
    if (protocol_->Stop() != boost::system::errc::success) {
      RCLCPP_FATAL(loop_node_->get_logger(), "stop failed");
      FAIL();
    }
    protocol_->Close();
  }

  void PrepareMainLoop() {
    network_ = boost::make_shared<hsrb_power_ecu::NetworkMock>();
    protocol_ = boost::make_shared<hsrb_power_ecu::PowerEcuProtocol>(network_, loop_node_);

    if (!protocol_->Open()) {
      RCLCPP_FATAL(loop_node_->get_logger(), "protocol_ Open failed.");
      FAIL();  // Currently, if network open fails, recovery is impossible
    }
    if (!protocol_->Init()) {
      RCLCPP_FATAL(loop_node_->get_logger(), "protocol_ Init Failed");
      FAIL();  // Currently, if network open fails, recovery is impossible
    }
    if (protocol_->Start() != boost::system::errc::success) {
      RCLCPP_FATAL(loop_node_->get_logger(), "start failed");
      FAIL();  // Currently, if network open fails, recovery is impossible
    }

    led_command_subscriber_ = std::make_shared<hsrb_power_ecu::LedCommandSubscriber>(loop_node_, protocol_);

    publishers_.push_back(std::make_shared<hsrb_power_ecu::BatteryStatePublisher>(loop_node_, protocol_));
    publishers_.push_back(std::make_shared<hsrb_power_ecu::RunstopStatePublisher>(loop_node_, protocol_));
    for (const auto& name : kParamAndTopicNames) {
      publishers_.push_back(std::make_shared<hsrb_power_ecu::BoolStatePublisher>(
          loop_node_, protocol_, name.first, name.second));
    }
  }

  void RunMainLoop(std::function<bool()> done_func) {
    diagnostic_updater::Updater diagnostic_updater(loop_node_, 0.1);
    diagnostic_updater.setHardwareID("hsrb_power_battery");
    hsrb_power_ecu::BatteryDiagnosticTask battery_diagnostic_task(loop_node_, protocol_);
    diagnostic_updater.add(battery_diagnostic_task);

    auto clock = loop_node_->get_clock();
    const auto start_time = clock->now();
    rclcpp::WallRate loop_rate(100.0);  // Hz
    while (rclcpp::ok() && !done_func()) {
      protocol_->ReceiveAll();
      protocol_->SendAll();
      for (auto& publisher : publishers_) {
        publisher->Publish();
      }
      rclcpp::spin_some(loop_node_);
      rclcpp::spin_some(test_node_);
      loop_rate.sleep();

      if (clock->now() - start_time > kLoopTimeout) {
        throw std::runtime_error("mainloop timeout");
      }
    }
  }

  // Used in main_loop
  rclcpp::Node::SharedPtr loop_node_;
  boost::shared_ptr<hsrb_power_ecu::NetworkMock> network_;
  boost::shared_ptr<hsrb_power_ecu::PowerEcuProtocol> protocol_;
  std::shared_ptr<hsrb_power_ecu::LedCommandSubscriber> led_command_subscriber_;
  std::vector<std::shared_ptr<hsrb_power_ecu::IStatePublisher>> publishers_;

  // Used in tests
  rclcpp::Node::SharedPtr test_node_;
};


TEST_F(MainLoopTest, LedCommandSubscribeTest) {
  auto command_status_led_publisher_ =
      test_node_->create_publisher<std_msgs::msg::ColorRGBA>("command_status_led_rgb", 1);

  std_msgs::msg::ColorRGBA message;
  message.r = 1.0;
  message.g = 1.0;
  message.b = 1.0;
  message.a = 1.0;
  network_->ResetSendBuffer();
  command_status_led_publisher_->publish(message);

  std::function<bool()> is_buffer_sent = [&] {return (network_->GetSendBuffer() != "");};
  EXPECT_NO_THROW(RunMainLoop(is_buffer_sent));
  ASSERT_TRUE((network_->GetSendBuffer().find("H,ledc_,023,255,255,255,") != -1));

  message.g = 0.0;
  message.a = 0.0;
  network_->ResetSendBuffer();
  command_status_led_publisher_->publish(message);

  EXPECT_NO_THROW(RunMainLoop(is_buffer_sent));
  ASSERT_TRUE((network_->GetSendBuffer().find("H,ledc_,023,255,000,255,") != -1));

  // Commands are not sent even if subscribing to the same message as the current state
  network_->ResetSendBuffer();
  command_status_led_publisher_->publish(message);

  EXPECT_THROW(RunMainLoop(is_buffer_sent), std::runtime_error);
  ASSERT_TRUE((network_->GetSendBuffer().find("H,ledc_,023,255,000,255,") == -1));

  // Expected input values are 0~1, multiplied by 255 and converted to int as command values
  // If command values are outside the range of 0~255, they are clipped to the maximum or minimum value within the range
  message.r = -10.0;
  message.g = 10.0;
  network_->ResetSendBuffer();
  command_status_led_publisher_->publish(message);

  EXPECT_NO_THROW(RunMainLoop(is_buffer_sent));
  ASSERT_TRUE((network_->GetSendBuffer().find("H,ledc_,023,000,255,255,") != -1));
}

TEST_F(MainLoopTest, PublishRateTest) {
  auto runstop_checker = std::make_shared<BoolPublishChecker>(test_node_, "runstop_button");

  std::string header = "E,ecu1_,326,";
  std::string body = ("0000000000,00000000000000,h00,h0000000000000000,"
                      "h00000000000000000000000000000000"
                      "00000000000000000000000000000000, 00000, 00000, 00000,"
                      " 00000, 000,h0000,00000,h0000,051,h00,h0000000000000000, "
                      "0000000000,-0000000000, 0000000000, 0000000000,-0000000000, "
                      "0000000000, 0000000000,-0000000000, 0000000000,-0000000000,"
                      "h00,");
  network_->UpdateBuffer(header + body);
  runstop_checker->Reset();

  const auto start_time = loop_node_->get_clock()->now();
  std::function<bool()> Timeout = [&] {
    return (loop_node_->get_clock()->now() - start_time > rclcpp::Duration::from_seconds(0.5));
  };
  EXPECT_NO_THROW(RunMainLoop(Timeout));
  // Ideally 50, but implementation-dependent, so some deviation is acceptable
  EXPECT_GE(runstop_checker->GetCount(), 48);
  EXPECT_LE(runstop_checker->GetCount(), 50);
}

TEST_F(MainLoopTest, PublishedDataTest) {
  auto battery_state_checker =
      std::make_shared<BatteryStatePublishChecker>(test_node_, "battery_state");
  auto runstop_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "runstop_button");
  auto base_f_bumper_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "base_f_bumper_sensor");
  auto base_b_bumper_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "base_b_bumper_sensor");
  auto emergency_stop_button_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "emergency_stop_button");
  auto wireless_stop_button_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "wireless_stop_button");
  auto wireless_stop_enable_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "wireless_stop_enable");
  auto battery_charging_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "battery_charging");
  auto base_magnetic_sensor_1_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "base_magnetic_sensor_1");
  auto base_magnetic_sensor_2_checker =
      std::make_shared<BoolPublishChecker>(test_node_, "base_magnetic_sensor_2");

  auto diagnostic_checker =
      std::make_shared<DiagnosticsChecker>(test_node_, "Battery updater");

  std::vector<std::shared_ptr<IPublishChecker>> publish_checkers = {
    battery_state_checker, runstop_checker, base_f_bumper_checker, base_b_bumper_checker, emergency_stop_button_checker,
    wireless_stop_button_checker, wireless_stop_enable_checker, battery_charging_checker,
    base_magnetic_sensor_1_checker, base_magnetic_sensor_2_checker, diagnostic_checker
  };

  std::string header = "E,ecu1_,326,";
  std::string body = ("0000000000,00000000000000,h00,h0000000000000000,"
                      "h00000000000000000000000000000000"
                      "00000000000000000000000000000000, 00000, 00000,-01000,"
                      " 00000, 000,h0000,00000,h0000,051,h00,h0000000000000000, "
                      "0000000000,-0000000000, 0000000000, 0000000000,-0000000000, "
                      "0000000000, 0000000000,-0000000000, 0000000000,-0000000000,"
                      "h00,");
  network_->UpdateBuffer(header + body);
  for (auto& publish_checker : publish_checkers) {publish_checker->Reset();}
  std::function<bool()> CheckUpdated = [&] {
    for (auto& publish_checker : publish_checkers) {
      if (!publish_checker->CheckUpdated()) return false;
    }
    return true;
  };
  EXPECT_NO_THROW(RunMainLoop(CheckUpdated));

  ASSERT_TRUE(runstop_checker->CheckExpected(true));
  ASSERT_TRUE(base_f_bumper_checker->CheckExpected(false));
  ASSERT_TRUE(base_b_bumper_checker->CheckExpected(false));
  ASSERT_TRUE(emergency_stop_button_checker->CheckExpected(true));
  ASSERT_TRUE(wireless_stop_button_checker->CheckExpected(true));
  ASSERT_TRUE(wireless_stop_enable_checker->CheckExpected(false));
  ASSERT_TRUE(battery_charging_checker->CheckExpected(false));
  ASSERT_TRUE(base_magnetic_sensor_1_checker->CheckExpected(false));
  ASSERT_TRUE(base_magnetic_sensor_2_checker->CheckExpected(false));

  auto value = battery_state_checker->GetValue();
  ASSERT_DOUBLE_EQ(value.capacity, 0.0);
  ASSERT_DOUBLE_EQ(value.charge, 0.0);
  ASSERT_DOUBLE_EQ(value.current, 1.0);
  ASSERT_DOUBLE_EQ(value.voltage, 0.0);
  ASSERT_DOUBLE_EQ(value.temperature, 0.0);

  ASSERT_EQ(diagnostic_checker->GetLevel(), diagnostic_msgs::msg::DiagnosticStatus::OK);
  ASSERT_EQ(diagnostic_checker->GetMessage(), "Battery Level: 51.000000 %");
  ASSERT_EQ(diagnostic_checker->GetHardwareId(), "hsrb_power_battery");
  ASSERT_EQ(diagnostic_checker->GetValue("full_charge_capacity"), "0");
  ASSERT_EQ(diagnostic_checker->GetValue("remaining_charge"), "0");
  ASSERT_EQ(diagnostic_checker->GetValue("electric_current"), "1");
  ASSERT_EQ(diagnostic_checker->GetValue("voltage"), "0");
  ASSERT_EQ(diagnostic_checker->GetValue("temperature"), "0");
  ASSERT_EQ(diagnostic_checker->GetValue("zero_percent_detected"), "False");
  ASSERT_EQ(diagnostic_checker->GetValue("discharge_enabled"), "False");
  ASSERT_EQ(diagnostic_checker->GetValue("over_discharge"), "False");
  ASSERT_EQ(diagnostic_checker->GetValue("full_charge"), "False");
  ASSERT_EQ(diagnostic_checker->GetValue("learning_enabled"), "False");
  ASSERT_EQ(diagnostic_checker->GetValue("triple_parallel"), "False");
  ASSERT_EQ(diagnostic_checker->GetValue("over_charge"), "False");
  ASSERT_EQ(diagnostic_checker->GetValue("relative_capacity"), "51");

  // When changing the corresponding parts, check if the published items are correctly updated
  body = ("0000000000,00000000000000,h00,h0000000000000063,"
          "h00000000000000000000000000000000"
          "00000000000000000000000000000000, 01000, 02000, 03000,"
          " 04000, 005,h00FF,00000,h0000,021,h63,h0000000000000000, "
          "0000000000,-0000000000, 0000000000, 0000000000,-0000000000, "
          "0000000000, 0000000000,-0000000000, 0000000000,-0000000000,"
          "h00,");
  network_->UpdateBuffer(header + body);
  for (auto& publish_checker : publish_checkers) {publish_checker->Reset();}
  EXPECT_NO_THROW(RunMainLoop(CheckUpdated));

  ASSERT_TRUE(runstop_checker->CheckExpected(false));
  ASSERT_TRUE(base_f_bumper_checker->CheckExpected(true));
  ASSERT_TRUE(base_b_bumper_checker->CheckExpected(true));
  ASSERT_TRUE(emergency_stop_button_checker->CheckExpected(false));
  ASSERT_TRUE(wireless_stop_button_checker->CheckExpected(false));
  ASSERT_TRUE(wireless_stop_enable_checker->CheckExpected(true));
  ASSERT_TRUE(battery_charging_checker->CheckExpected(true));
  ASSERT_TRUE(base_magnetic_sensor_1_checker->CheckExpected(true));
  ASSERT_TRUE(base_magnetic_sensor_2_checker->CheckExpected(true));

  value = battery_state_checker->GetValue();
  ASSERT_DOUBLE_EQ(value.capacity, 1.0);
  ASSERT_DOUBLE_EQ(value.charge, 2.0);
  ASSERT_DOUBLE_EQ(value.current, -3.0);
  ASSERT_DOUBLE_EQ(value.voltage, 4.0);
  ASSERT_DOUBLE_EQ(value.temperature, 5.0);

  ASSERT_EQ(diagnostic_checker->GetLevel(), diagnostic_msgs::msg::DiagnosticStatus::WARN);
  ASSERT_EQ(diagnostic_checker->GetMessage(), "Battery Level: 21.000000 %");
  ASSERT_EQ(diagnostic_checker->GetHardwareId(), "hsrb_power_battery");
  ASSERT_EQ(diagnostic_checker->GetValue("full_charge_capacity"), "1");
  ASSERT_EQ(diagnostic_checker->GetValue("remaining_charge"), "2");
  ASSERT_EQ(diagnostic_checker->GetValue("electric_current"), "-3");
  ASSERT_EQ(diagnostic_checker->GetValue("voltage"), "4");
  ASSERT_EQ(diagnostic_checker->GetValue("temperature"), "5");
  ASSERT_EQ(diagnostic_checker->GetValue("zero_percent_detected"), "True");
  ASSERT_EQ(diagnostic_checker->GetValue("discharge_enabled"), "True");
  ASSERT_EQ(diagnostic_checker->GetValue("over_discharge"), "True");
  ASSERT_EQ(diagnostic_checker->GetValue("full_charge"), "True");
  ASSERT_EQ(diagnostic_checker->GetValue("learning_enabled"), "True");
  ASSERT_EQ(diagnostic_checker->GetValue("triple_parallel"), "True");
  ASSERT_EQ(diagnostic_checker->GetValue("over_charge"), "True");
  ASSERT_EQ(diagnostic_checker->GetValue("relative_capacity"), "21");

  // Battery diagnostics have detailed conditions, so check each condition
  body = ("0000000000,00000000000000,h00,h0000000000000063,"
          "h00000000000000000000000000000000"
          "00000000000000000000000000000000, 01000, 02000, 03000,"
          " 04000, 005,h00FF,00000,h0000,019,h60,h0000000000000000, "
          "0000000000,-0000000000, 0000000000, 0000000000,-0000000000, "
          "0000000000, 0000000000,-0000000000, 0000000000,-0000000000,"
          "h00,");
  network_->UpdateBuffer(header + body);
  for (auto& publish_checker : publish_checkers) {publish_checker->Reset();}
  EXPECT_NO_THROW(RunMainLoop(CheckUpdated));

  ASSERT_EQ(diagnostic_checker->GetLevel(), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  ASSERT_EQ(diagnostic_checker->GetValue("relative_capacity"), "19");

  body = ("0000000000,00000000000000,h00,h0000000000000063,"
          "h00000000000000000000000000000000"
          "00000000000000000000000000000000, 01000, 02000,-01000,"
          " 04000, 005,h00FF,00000,h0000,019,h60,h0000000000000000, "
          "0000000000,-0000000000, 0000000000, 0000000000,-0000000000, "
          "0000000000, 0000000000,-0000000000, 0000000000,-0000000000,"
          "h00,");
  network_->UpdateBuffer(header + body);
  for (auto& publish_checker : publish_checkers) {publish_checker->Reset();}
  EXPECT_NO_THROW(RunMainLoop(CheckUpdated));

  ASSERT_EQ(diagnostic_checker->GetLevel(), diagnostic_msgs::msg::DiagnosticStatus::OK);
  ASSERT_EQ(diagnostic_checker->GetValue("relative_capacity"), "19");
}
}  // namespace hsrb_power_ecu

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}
