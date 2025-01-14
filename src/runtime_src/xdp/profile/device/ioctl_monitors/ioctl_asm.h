/*
 * Copyright (C) 2020-2022 Xilinx Inc - All rights reserved
 * Xilinx Debug & Profile (XDP) APIs
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You may
 * not use this file except in compliance with the License. A copy of the
 * License is located at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef XDP_PROFILE_DEVICE_IOCTL_ASM_H
#define XDP_PROFILE_DEVICE_IOCTL_ASM_H

#include "xdp/profile/device/asm.h"

namespace xdp {

/** Specialization for ASM Profile IP with support for open+ioctl on device driver based ASM subdevice
 */

class IOCtlASM : public ASM
{
public:
  IOCtlASM(Device*  handle  /* XDP Device Abstraction handle : xrt or HAL device handle */
            ,uint64_t index   /* Index of the IP in Debug_IP_Layout */
            ,uint64_t instIdx /* Instance Index of this Profile IP type */
            ,debug_ip_data* data = nullptr);

  virtual ~IOCtlASM();

  virtual size_t startCounter();
  virtual size_t stopCounter();
  virtual size_t readCounter(xclCounterResults& counterResult);

  virtual size_t triggerTrace(uint32_t traceOption /*startTrigger*/);

  virtual int read(uint64_t offset, size_t size, void* data);
  virtual int write(uint64_t offset, size_t size, void* data);

  virtual bool isOpened();

protected:
  uint64_t instance_index;
  int      driver_FD = -1;
};

}

#endif
