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

#include "hsrb_cgos_driver/hsrb_cgos_lib.hpp"

namespace {

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

// gpio directory path
const char* const kGpioPath = "/dev/cgos";

const uint32_t kCgosIoctlCode = 44444;  // Cgos request code

const uint32_t kCgosDrvVerMajor = 1;

const uint32_t kCgosDrvGetVerion = 0;
const uint32_t kCgosBoardClose = 1;
const uint32_t kCgosBoardCount = 2;
const uint32_t kCgosBoardOpen = 3;
const uint32_t kCgosIORead = 41;
const uint32_t kCgosIOWrite = 42;
}  // anonymous namespace

namespace hsrb_cgos_driver {

HsrbCgosLib::HsrbCgosLib()
    : fd_(-1),
      last_error_(0),
      system_(std::make_shared<System>()) {}

HsrbCgosLib::HsrbCgosLib(std::shared_ptr<ISystemInterface> system)
    : fd_(-1),
      last_error_(0),
      system_(system) {}

HsrbCgosLib::~HsrbCgosLib() {
  Finalize();
}

bool HsrbCgosLib::Initialize() {
  // cgos initialization
  int32_t fd;
  uint32_t version;

  fd = system_->Open(kGpioPath, O_WRONLY);
  if (fd >= 0) {
    fd_ = fd;

    // version check
    if (CgosDriver(kCgosDrvGetVerion, 0, 0, 0, 0, 0, 0, &version, NULL)) {
      if ((version >> 24) == kCgosDrvVerMajor) {
        return true;
      } else {
        Finalize();
      }
    }
  }

  return false;
}

bool HsrbCgosLib::Finalize() {
  if (fd_ >= 0) {
    system_->Close(fd_);

    fd_ = -1;
    return true;
  }

  return false;
}

bool HsrbCgosLib::BoardOpen(uint32_t board_class, uint32_t num, uint32_t flags, uint32_t* handle) {
  return CgosDriver(kCgosBoardOpen, 0, num, board_class, flags, 0, 0, handle, NULL);
}

bool HsrbCgosLib::BoardClose(uint32_t handle) {
  return CgosDriver(kCgosBoardClose, handle, 0, 0, 0, 0, 0, NULL, NULL);
}

bool HsrbCgosLib::GetIOCount(uint32_t handle, uint32_t* io_count) {
  return CgosDriver(kCgosBoardCount, handle, 0, 0, 0, 0, 0, io_count, NULL);
}

bool HsrbCgosLib::IORead(uint32_t handle, uint32_t unit, uint32_t* read_data) {
  return CgosDriver(kCgosIORead, handle, unit, 0, 0, 0, 0, read_data, NULL);
}

bool HsrbCgosLib::IOWrite(uint32_t handle, uint32_t unit, uint32_t write_data) {
  return CgosDriver(kCgosIOWrite, handle, unit, write_data, 0, 0, 0, NULL, NULL);
}

bool HsrbCgosLib::CgosDriver(uint32_t fct, uint32_t handle, uint32_t type,
  uint32_t par0, uint32_t par1, uint32_t par2, uint32_t par3, uint32_t* pret0, uint32_t* pret1) {
  cgosctrl_in ctrl_in;
  cgosctrl_out ctrl_out;
  ioctl_desc iodb;
  uint32_t cb = 0;
  int32_t ret;

  ctrl_in.fct = fct;
  ctrl_in.handle = handle;
  ctrl_in.type = type;
  ctrl_in.pars[0] = par0;
  ctrl_in.pars[1] = par1;
  ctrl_in.pars[2] = par2;
  ctrl_in.pars[3] = par3;

  iodb.pInBuffer = &ctrl_in;
  iodb.nInBufferSize = sizeof(ctrl_in);
  iodb.pOutBuffer = &ctrl_out;
  iodb.nOutBufferSize = sizeof(ctrl_out);
  iodb.pBytesReturned = &cb;

  ret = system_->Ioctl(fd_, kCgosIoctlCode, &iodb);

  if ((ret < 0) || (cb < sizeof(cgosctrl_out))) {
    return false;
  }
  if (ctrl_out.status) {
    last_error_ = ctrl_out.status;
    return false;
  }
  if (pret0) {
    *pret0 = ctrl_out.rets[0];
  }
  if (pret1) {
    *pret1 = ctrl_out.rets[1];
  }

  return true;
}

}  // namespace hsrb_cgos_driver
