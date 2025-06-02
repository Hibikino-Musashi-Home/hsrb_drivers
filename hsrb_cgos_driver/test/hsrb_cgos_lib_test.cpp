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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <hsrb_cgos_driver/hsrb_cgos_lib.hpp>

namespace hsrb_cgos_driver {

struct cgosctrl_in {
  uint32_t fct;
  uint32_t handle;
  uint32_t type;
  uint32_t pars[4];
};

struct cgosctrl_out {
  uint32_t status;
  uint32_t rets[2];
};

struct ioctl_desc {
  void *pInBuffer;
  uint32_t nInBufferSize;
  void *pOutBuffer;
  uint32_t nOutBufferSize;
  uint32_t *pBytesReturned;
};

class SystemInterfaceMock : public ISystemInterface {
 public:
  SystemInterfaceMock() {}
  MOCK_METHOD(int, Open, (const char *file, int oflag), (override));
  MOCK_METHOD(int, Close, (int fd), (override));
  MOCK_METHOD(int, Ioctl, (int fd, unsigned int request, void *argp), (override));

  // Ioctl success
  int IoctrlSuccess(int fd, unsigned int request, void *argp) {
    return IoctrlSetRet(fd, request, argp, 0);
  }

  // Get Ioctl version
  int GetVerion(int fd, unsigned int request, void *argp) {
    return IoctrlSetRet(fd, request, argp, 0x01000000);
  }

  // Ioctl BoardOpen
  int BoardOpen(int fd, unsigned int request, void *argp) {
    return IoctrlSetRet(fd, request, argp, 1);
  }

  // Ioctl GetIOCount
  int GetIOCount(int fd, unsigned int request, void *argp) {
    return IoctrlSetRet(fd, request, argp, 2);
  }

  // Ioctl IORead
  int IORead(int fd, unsigned int request, void *argp) {
    return IoctrlSetRet(fd, request, argp, 0x0000000F);
  }

 private:
  // Set value to Ioctl rets[0]
  int IoctrlSetRet([[maybe_unused]] int fd, [[maybe_unused]] unsigned int request, void *argp, uint32_t ret) {
    ioctl_desc* iodb = reinterpret_cast<ioctl_desc *>(argp);
    *iodb->pBytesReturned = sizeof(cgosctrl_out);
    cgosctrl_out* ctrl_out = reinterpret_cast<cgosctrl_out *>(iodb->pOutBuffer);

    ctrl_out->status = 0;
    ctrl_out->rets[0] = ret;

    return 0;
  }

  System system_;
};

class HsrbCgosLibTest : public ::testing::Test {
 public:
  HsrbCgosLibTest() : mock_(std::make_shared<SystemInterfaceMock>()), cgos_(mock_) {}

 public:
  std::shared_ptr<SystemInterfaceMock> mock_;
  HsrbCgosLib cgos_;  // Test subject
};

// Test of Initialize()
TEST_F(HsrbCgosLibTest, Initialize) {
  // Initialize success
  EXPECT_CALL(*mock_, Open(::testing::_, ::testing::_)).WillOnce(testing::Return(0));
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_))
    .WillOnce(::testing::Invoke(mock_.get(), &SystemInterfaceMock::GetVerion));
  EXPECT_TRUE(cgos_.Initialize());

  // Initialize failure (Open failure)
  EXPECT_CALL(*mock_, Open(::testing::_, ::testing::_)).WillOnce(testing::Return(-1));
  EXPECT_FALSE(cgos_.Initialize());

  // Initialize failure (Ioctl failure)
  EXPECT_CALL(*mock_, Open(::testing::_, ::testing::_)).WillOnce(testing::Return(0));
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_)).WillOnce(testing::Return(-1));
  EXPECT_FALSE(cgos_.Initialize());

  // Initialize failure (version abnormal)
  EXPECT_CALL(*mock_, Open(::testing::_, ::testing::_)).WillOnce(testing::Return(0));
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_));
  EXPECT_FALSE(cgos_.Initialize());
}

// Test of Finalize()
TEST_F(HsrbCgosLibTest, Finalize) {
  // If not opened, Close is not called
  EXPECT_CALL(*mock_, Close(::testing::_)).Times(0);
  EXPECT_FALSE(cgos_.Finalize());

  EXPECT_CALL(*mock_, Open(::testing::_, ::testing::_)).WillOnce(testing::Return(0));
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_))
    .WillOnce(::testing::Invoke(mock_.get(), &SystemInterfaceMock::GetVerion));
  EXPECT_TRUE(cgos_.Initialize());

  // If opened, Close is called
  EXPECT_CALL(*mock_, Close(::testing::_)).Times(1);
  EXPECT_TRUE(cgos_.Finalize());
}

// Test of BoardOpen()
TEST_F(HsrbCgosLibTest, BoardOpen) {
  uint32_t handle = 0;

  // BoardOpen success
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_))
    .WillOnce(::testing::Invoke(mock_.get(), &SystemInterfaceMock::BoardOpen));
  EXPECT_TRUE(cgos_.BoardOpen(kCgosBoardClassDefault, 0, kCgosBoardOpenFlagsDefault, &handle));
  EXPECT_EQ(handle, 1);

  // BoardOpen failure
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_));
  EXPECT_FALSE(cgos_.BoardOpen(kCgosBoardClassDefault, 0, kCgosBoardOpenFlagsDefault, &handle));
}

// Test of BoardClose()
TEST_F(HsrbCgosLibTest, BoardClose) {
  uint32_t handle = 0;

  // BoardClose success
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_))
    .WillOnce(::testing::Invoke(mock_.get(), &SystemInterfaceMock::IoctrlSuccess));
  EXPECT_TRUE(cgos_.BoardClose(handle));

  // BoardClose failure
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_));
  EXPECT_FALSE(cgos_.BoardClose(handle));
}

// Test of GetIOCount()
TEST_F(HsrbCgosLibTest, GetIOCount) {
  uint32_t handle = 0;
  uint32_t io_count = 0;

  // GetIOCount success
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_))
    .WillOnce(::testing::Invoke(mock_.get(), &SystemInterfaceMock::GetIOCount));
  EXPECT_TRUE(cgos_.GetIOCount(handle, &io_count));
  EXPECT_EQ(io_count, 2);

  // GetIOCount failure
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_));
  EXPECT_FALSE(cgos_.GetIOCount(handle, &io_count));
}

// Test of IORead()
TEST_F(HsrbCgosLibTest, IORead) {
  uint32_t handle = 0;
  uint32_t io_count = 0;
  uint32_t read_data = 0;

  // IORead success
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_))
    .WillOnce(::testing::Invoke(mock_.get(), &SystemInterfaceMock::IORead));
  EXPECT_TRUE(cgos_.IORead(handle, io_count, &read_data));
  EXPECT_EQ(read_data, 0x0000000F);

  // IORead failure
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_));
  EXPECT_FALSE(cgos_.IORead(handle, io_count, &read_data));
}

// Test of IOWrite()
TEST_F(HsrbCgosLibTest, IOWrite) {
  uint32_t handle = 0;
  uint32_t io_count = 0;
  uint32_t write_data = 0;

  // IOWrite success
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_))
    .WillOnce(::testing::Invoke(mock_.get(), &SystemInterfaceMock::IoctrlSuccess));
  EXPECT_TRUE(cgos_.IOWrite(handle, io_count, write_data));

  // IOWrite failure
  EXPECT_CALL(*mock_, Ioctl(::testing::_, ::testing::_, ::testing::_));
  EXPECT_FALSE(cgos_.IOWrite(handle, io_count, write_data));
}

}  // namespace hsrb_cgos_driver

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
