/**
 * Copyright (C) 2022 Contemporary Software (Chris Johns)
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

#include <stdarg.h>

#include <iostream>
#include <thread>

#include <core/common/config_reader.h>
#include <core/common/message.h>
#include <core/common/time.h>

#include "gen/version.h"

#include "device_rtems.h"
#include "system_rtems.h"

#include <rtems.h>
#include <rtems/score/protectedheap.h>
#include <rtems/score/wkspace.h>

namespace {
// Singleton registers with base class xrt_core::system
// during static global initialization.  If statically
// linking with libxrt_core, then explicit initialiation
// is required
static xrt_core::system_rtems*
singleton_instance()
{
  static xrt_core::system_rtems singleton;
  return &singleton;
}
};

namespace xrt_core {

void
system_rtems::
get_xrt_info(boost::property_tree::ptree &pt)
{
  pt.put("build.version", xrt_build_version);
  pt.put("build.hash", xrt_build_version_hash);
  pt.put("build.date", xrt_build_version_date);
  pt.put("build.branch", xrt_build_version_branch);
}

void
system_rtems::
get_os_info(boost::property_tree::ptree &pt)
{
  pt.put("sysname", "RTEMS");
  pt.put("release", "unreleased");
  pt.put("version", rtems_get_version_string());
  pt.put("machine", "xilinx_versal_lp64_vck190");

  pt.put("distribution", "Contemporary Software");
  pt.put("model", "Versal");
  pt.put("cores", std::thread::hardware_concurrency());

  Heap_Information_block heap_info;
  _Protected_heap_Get_information(&_Workspace_Area, &heap_info);
  auto total_mem = heap_info.Free.total + heap_info.Used.total;
  pt.put("memory_bytes", (boost::format("0x%lx") % (total_mem)).str());

  pt.put("now", xrt_core::timestamp());
}

std::pair<device::id_type, device::id_type>
system_rtems::
get_total_devices(bool is_user) const
{
  device::id_type num = xclProbe();
  return std::make_pair(num, num);
}

void
system_rtems::
scan_devices(bool , bool ) const
{
}

std::shared_ptr<device>
system_rtems::
get_userpf_device(device::id_type id) const
{
  return xrt_core::get_userpf_device(xclOpen(id, nullptr, XCL_QUIET));
}

std::shared_ptr<device>
system_rtems::
get_userpf_device(device::handle_type handle, device::id_type id) const
{
  // deliberately not using std::make_shared (used with weak_ptr)
  return std::shared_ptr<device_rtems>(new device_rtems(handle, id, true));
}

std::shared_ptr<device>
system_rtems::
get_mgmtpf_device(device::id_type id) const
{
  // deliberately not using std::make_shared (used with weak_ptr)
  return std::shared_ptr<device_rtems>(new device_rtems(nullptr, id, false));
}

void
system_rtems::
program_plp(const device* dev, const std::vector<char> &buffer) const
{
  throw std::runtime_error("plp program is not supported");
}

void
system_rtems::
mem_read(const device*, long long, long long, std::string) const
{
  throw std::runtime_error("memory read is not supported");
}

namespace edge_rtems {

std::shared_ptr<device>
get_userpf_device(device::handle_type device_handle, device::id_type id)
{
  singleton_instance(); // force loading if necessary
  return xrt_core::get_userpf_device(device_handle, id);
}

} // edge_rtems
} // xrt_core
