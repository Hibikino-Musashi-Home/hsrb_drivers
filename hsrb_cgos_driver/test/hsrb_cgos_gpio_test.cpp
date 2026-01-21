/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>

#include <hsrb_cgos_driver/hsrb_cgos_hw.hpp>
#include <hsrb_cgos_driver/hsrb_cgos_lib.hpp>

namespace hsrb_cgos_driver {

class CgosMock : public ICgosInterface {
 public:
  MOCK_METHOD(bool, Initialize, (), (override));
  MOCK_METHOD(bool, Finalize, (), (override));
  MOCK_METHOD(bool, BoardOpen, (uint32_t, uint32_t, uint32_t, uint32_t*), (override));
  MOCK_METHOD(bool, BoardClose, (uint32_t), (override));
  MOCK_METHOD(bool, GetIOCount, (uint32_t, uint32_t*), (override));
  MOCK_METHOD(bool, IORead, (uint32_t, uint32_t, uint32_t*), (override));
  MOCK_METHOD(bool, IOWrite, (uint32_t, uint32_t, uint32_t), (override));
};

class HsrbCgosHwDeathTest : public ::testing::Test {
 protected:
  void SetUp() override {
    cgos_mock_ = std::make_shared<CgosMock>();
    hardware_info_.name = "hsrb_cgos_hw";
    gpio_info_.name = "pin0";

    hw_ = std::make_shared<HsrbCgosHw>(cgos_mock_);
  }

  std::shared_ptr<CgosMock> cgos_mock_;
  hardware_interface::HardwareInfo hardware_info_;
  hardware_interface::ComponentInfo gpio_info_;
  std::shared_ptr<HsrbCgosHw> hw_;
};

// on_init successful
TEST_F(HsrbCgosHwDeathTest, on_init) {
  gpio_info_.parameters["pin"] = "0";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  ASSERT_EQ(hw_->on_init(hardware_info_), hardware_interface::CallbackReturn::SUCCESS);
}

// on_configure successful
TEST_F(HsrbCgosHwDeathTest, on_configure) {
  gpio_info_.parameters["pin"] = "0";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  hw_->on_init(hardware_info_);

  EXPECT_CALL(*cgos_mock_, Initialize()).WillOnce(::testing::Return(true));
  EXPECT_CALL(*cgos_mock_, BoardOpen(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .WillOnce(::testing::Return(true));
  EXPECT_CALL(*cgos_mock_, GetIOCount(::testing::_, ::testing::_))
    .WillOnce(::testing::DoAll(::testing::SetArgPointee<1>(1), ::testing::Return(true)));

  ASSERT_EQ(hw_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
}

// Initialize failed on successful on_configure (returns SUCCESS)
TEST_F(HsrbCgosHwDeathTest, OnConfigure_InitializeFails_ReturnsSuccess) {
  gpio_info_.parameters["pin"] = "0";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  hw_->on_init(hardware_info_);

  EXPECT_CALL(*cgos_mock_, Initialize()).WillOnce(::testing::Return(false));

  ASSERT_EQ(hw_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
}

// BoardOpen failed on successful on_configure (returns SUCCESS)
TEST_F(HsrbCgosHwDeathTest, OnConfigure_BoardOpenFails_ReturnsSuccess) {
  gpio_info_.parameters["pin"] = "0";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  hw_->on_init(hardware_info_);

  EXPECT_CALL(*cgos_mock_, Initialize()).WillOnce(::testing::Return(true));
  EXPECT_CALL(*cgos_mock_, BoardOpen(::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .WillOnce(::testing::Return(false));

  ASSERT_EQ(hw_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
}

// read successful
TEST_F(HsrbCgosHwDeathTest, read) {
  gpio_info_.parameters["pin"] = "0";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  hw_->on_init(hardware_info_);

  EXPECT_CALL(*cgos_mock_, Initialize()).WillOnce(::testing::Return(true));
  // Set a valid handle value (non-zero) for successful BoardOpen
  EXPECT_CALL(*cgos_mock_, BoardOpen(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .WillOnce(::testing::DoAll(::testing::SetArgPointee<3>(1), ::testing::Return(true)));
  // Set a valid handle value (non-zero) for successful GetIOCount
  EXPECT_CALL(*cgos_mock_, GetIOCount(::testing::_, ::testing::_))
      .WillOnce(::testing::DoAll(::testing::SetArgPointee<1>(1), ::testing::Return(true)));
  ASSERT_EQ(hw_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_CALL(*cgos_mock_, IORead(::testing::_, ::testing::_, ::testing::_))
      .WillOnce(::testing::DoAll(::testing::SetArgPointee<2>(1), ::testing::Return(true)));

  ASSERT_EQ(hw_->read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
}

// Skip read when cgos_handle is invalid
TEST_F(HsrbCgosHwDeathTest, ReadNoHandleSkip) {
  gpio_info_.parameters["pin"] = "0";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  hw_->on_init(hardware_info_);

  EXPECT_CALL(*cgos_mock_, Initialize()).WillOnce(::testing::Return(false));
  EXPECT_CALL(*cgos_mock_, IORead).Times(0);

  ASSERT_EQ(hw_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->read(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
}

// write successful
TEST_F(HsrbCgosHwDeathTest, write) {
  gpio_info_.parameters["pin"] = "0";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  hw_->on_init(hardware_info_);

  EXPECT_CALL(*cgos_mock_, Initialize()).WillOnce(::testing::Return(true));
  // Set a valid handle value (non-zero) for successful BoardOpen
  EXPECT_CALL(*cgos_mock_, BoardOpen(::testing::_, ::testing::_, ::testing::_, ::testing::_))
      .WillOnce(::testing::DoAll(::testing::SetArgPointee<3>(1), ::testing::Return(true)));
  // Set a valid handle value (non-zero) for successful GetIOCount
  EXPECT_CALL(*cgos_mock_, GetIOCount(::testing::_, ::testing::_))
      .WillOnce(::testing::DoAll(::testing::SetArgPointee<1>(1), ::testing::Return(true)));
  ASSERT_EQ(hw_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_CALL(*cgos_mock_, IOWrite(::testing::_, ::testing::_, ::testing::_)).WillOnce(::testing::Return(true));

  ASSERT_EQ(hw_->write(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
}

// Skip write when cgos_handle is invalid
TEST_F(HsrbCgosHwDeathTest, WriteNoHandleSkip) {
  gpio_info_.parameters["pin"] = "0";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  hw_->on_init(hardware_info_);

  EXPECT_CALL(*cgos_mock_, Initialize()).WillOnce(::testing::Return(false));
  EXPECT_CALL(*cgos_mock_, IOWrite).Times(0);

  ASSERT_EQ(hw_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->write(rclcpp::Time(), rclcpp::Duration(0, 0)), hardware_interface::return_type::OK);
}

// on_cleanup successful
TEST_F(HsrbCgosHwDeathTest, on_cleanup) {
  gpio_info_.parameters["pin"] = "0";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  hw_->on_init(hardware_info_);
  hw_->on_configure(rclcpp_lifecycle::State());

  EXPECT_CALL(*cgos_mock_, BoardClose(::testing::_)).Times(1);
  EXPECT_CALL(*cgos_mock_, Finalize()).Times(1);

  ASSERT_EQ(hw_->on_cleanup(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
}

// When empty
TEST_F(HsrbCgosHwDeathTest, EmptygpiosTest) {
  auto result = hw_->on_init(hardware_info_);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

// When pin is missing
TEST_F(HsrbCgosHwDeathTest, MissingPinParametersTest) {
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  auto result = hw_->on_init(hardware_info_);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

// When direction is missing
TEST_F(HsrbCgosHwDeathTest, MissingDirectionParametersTest) {
  gpio_info_.parameters["pin"] = "0";
  hardware_info_.gpios.push_back(gpio_info_);

  auto result = hw_->on_init(hardware_info_);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

// When pin is out of valid range for input
TEST_F(HsrbCgosHwDeathTest, InvalidPinInputRangeTest) {
  gpio_info_.parameters["pin"] = "4";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  auto result = hw_->on_init(hardware_info_);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

// When pin is out of valid range for output
TEST_F(HsrbCgosHwDeathTest, InvalidPinOutputRangeTest) {
  gpio_info_.parameters["pin"] = "8";
  gpio_info_.parameters["direction"] = "out";
  hardware_info_.gpios.push_back(gpio_info_);

  auto result = hw_->on_init(hardware_info_);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

// When pin has an invalid value
TEST_F(HsrbCgosHwDeathTest, PinParsingFailureTest) {
  gpio_info_.parameters["pin"] = "pin_ng";
  gpio_info_.parameters["direction"] = "in";
  hardware_info_.gpios.push_back(gpio_info_);

  auto result = hw_->on_init(hardware_info_);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

// When direction has an invalid value
TEST_F(HsrbCgosHwDeathTest, InvalidDirectionTest) {
  gpio_info_.parameters["pin"] = "0";
  gpio_info_.parameters["direction"] = "direction_ng";
  hardware_info_.gpios.push_back(gpio_info_);

  auto result = hw_->on_init(hardware_info_);
  EXPECT_EQ(result, hardware_interface::CallbackReturn::ERROR);
}

}  // namespace hsrb_cgos_driver

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
