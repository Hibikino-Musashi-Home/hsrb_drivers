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
#ifndef HSRB_CGOS_DRIVER_HSRB_CGOS_LIB_HPP_
#define HSRB_CGOS_DRIVER_HSRB_CGOS_LIB_HPP_

#include <memory>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

namespace hsrb_cgos_driver {

/// congatec's CGOS compliant board class
enum HsrbCgosLibBoardClass {
  // Default type
  kCgosBoardClassDefault = 0,
  // CPU type
  kCgosBoardClassCpu = 1,
  // VGA type
  kCgosBoardClassVga = 2,
  // IO type
  kCgosBoardClassIo = 4
};

/// Flags for opening a congatec's CGOS compliant board
enum HsrbCgosLibBoardOpenFlags {
  // Scan for matches with the specified board class
  kCgosBoardOpenFlagsDefault = 0,
  // Scan for matches with the Primary Class of the board
  kCgosBoardOpenFlagsPrimaryOnly = 1
};

/**
 * @brief Interface for embedded Linux functions
 * Extracted to facilitate mock testing
 */
class ISystemInterface {
 public:
  virtual int Open(const char *file, int oflag) = 0;
  virtual int32_t Close(int32_t fd) = 0;
  virtual int32_t Ioctl(int32_t fd, uint32_t request, void *argp) = 0;
};

/**
 * @brief Interface for CGOS control class
 * Extracted to facilitate mock testing
 */
class ICgosInterface {
 public:
  virtual bool Initialize() = 0;
  virtual bool Finalize() = 0;
  virtual bool BoardOpen(uint32_t board_class, uint32_t num, uint32_t flags, uint32_t* handle) = 0;
  virtual bool BoardClose(uint32_t handle) = 0;
  virtual bool GetIOCount(uint32_t handle, uint32_t* io_count) = 0;
  virtual bool IORead(uint32_t handle, uint32_t unit, uint32_t* read_data) = 0;
  virtual bool IOWrite(uint32_t handle, uint32_t unit, uint32_t write_data) = 0;
};

class System : public ISystemInterface {
 public:
  virtual int32_t Open(const char *file, int32_t oflag) {
    return ::open(file, oflag);
  }
  virtual int32_t Close(int32_t fd) {
    return ::close(fd);
  }
  virtual int32_t Ioctl(int32_t fd, uint32_t request, void *argp) {
    return ::ioctl(fd, request, argp);
  }
};

/// @brief CGOS control class
class HsrbCgosLib : public ICgosInterface {
 public:
  HsrbCgosLib();
  explicit HsrbCgosLib(std::shared_ptr<ISystemInterface> system);
  virtual ~HsrbCgosLib();
  virtual bool Initialize();
  virtual bool Finalize();
  virtual bool BoardOpen(uint32_t board_class, uint32_t num, uint32_t flags, uint32_t* handle);
  virtual bool BoardClose(uint32_t handle);
  virtual bool GetIOCount(uint32_t handle, uint32_t* io_count);
  virtual bool IORead(uint32_t handle, uint32_t unit, uint32_t* read_data);
  virtual bool IOWrite(uint32_t handle, uint32_t unit, uint32_t write_data);

 private:
  bool CgosDriver(uint32_t fct, uint32_t handle, uint32_t type,
                  uint32_t par0, uint32_t par1, uint32_t par2, uint32_t par3,
                  uint32_t* pret0, uint32_t* pret1);

  int32_t fd_;  // File descriptor
  uint32_t last_error_;
  std::shared_ptr<ISystemInterface> system_;
};

}  // namespace hsrb_cgos_driver

#endif  /// HSRB_CGOS_DRIVER_HSRB_CGOS_LIB_HPP_
