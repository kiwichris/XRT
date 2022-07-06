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

#if USE_SHIM
unsigned xclProbe() {
  return 0;
}

xclDeviceHandle xclOpen(
  unsigned deviceIndex, const char*, xclVerbosityLevel) {
  std::cout << "xclOpen: " << std::endl;
  return static_cast<xclDeviceHandle>(XRT_NULL_HANDLE);
}

void xclClose(xclDeviceHandle handle) {
  std::cout << "xclClose: " << handle << std::endl;
}

unsigned int xclAllocBO(
  xclDeviceHandle handle, size_t size, int unused, unsigned flags) {
  std::cout << "xclAllocBO: " << handle << std::endl;
  return static_cast<unsigned int>(-EINVAL);
}

unsigned int xclAllocUserPtrBO(
  xclDeviceHandle handle, void *userptr, size_t size, unsigned flags) {
  std::cout << "xclAllocUserPtrBO: " << handle << std::endl;
  return static_cast<unsigned int>(-EINVAL);
}

unsigned int xclGetHostBO(
  xclDeviceHandle handle, uint64_t paddr, size_t size) {
  std::cout << "xclGetHostBO: " << handle << std::endl;
  return -EINVAL;
}

void xclFreeBO(xclDeviceHandle handle, unsigned int boHandle) {
  std::cout << "xclFreeBO:" << handle << std::endl;
}

size_t xclWriteBO(
  xclDeviceHandle handle, unsigned int boHandle, const void *src,
  size_t size, size_t seek) {
  std::cout << "xclWriteBO:" << handle << std::endl;
  return -EINVAL;
}

size_t xclReadBO(
  xclDeviceHandle handle, unsigned int boHandle, void *dst,
  size_t size, size_t skip) {
  std::cout << "xclReadBO:" << handle << std::endl;
  return -EINVAL;
}

void* xclMapBO(
  xclDeviceHandle handle, unsigned int boHandle, bool write) {
  std::cout << "xclMapBO:" << handle << std::endl;
  return static_cast<void*>(nullptr);
}

int xclUnmapBO(
  xclDeviceHandle handle, unsigned int boHandle, void* addr) {
  std::cout << "xclUnmapBO:" << handle << std::endl;
  return -EINVAL;
}

int xclSyncBO(
  xclDeviceHandle handle, unsigned int boHandle, xclBOSyncDirection dir,
  size_t size, size_t offset) {
  std::cout << "xclSyncBO:" << handle << std::endl;
  return -EINVAL;
}

int xclCopyBO(
  xclDeviceHandle handle, unsigned int dst_boHandle,
  unsigned int src_boHandle, size_t size, size_t dst_offset, size_t src_offset) {
  std::cout << "xclCopyBO:" << handle << std::endl;
  return -EINVAL;
}

int xclExportBO(xclDeviceHandle handle, unsigned int boHandle) {
  std::cout << "xclExportBO: " << handle << std::endl;
  return -EINVAL;
}

unsigned int xclImportBO(xclDeviceHandle handle, int fd, unsigned flags) {
  std::cout << "xclImportBO: " << handle << std::endl;
  return -EINVAL;
}

int xclCloseExportHandle(int fd) {
  std::cout << "xclCloseExportHandle: " << fd << std::endl;
  return 0;
}

static int xclLoadXclBinImpl(
  xclDeviceHandle handle, const xclBin *buffer, bool meta){
  std::cout << "xclLoadXclBinImpl: " << handle << " meta: " << meta << std::endl;
  return 1;
}

int xclLoadXclBinMeta(xclDeviceHandle handle, const xclBin *buffer) {
  return xclLoadXclBinImpl(handle, buffer, true);
}

int xclLoadXclBin(xclDeviceHandle handle, const xclBin *buffer) {
  return xclLoadXclBinImpl(handle, buffer, false);
}

size_t xclWrite(
  xclDeviceHandle handle, xclAddressSpace space, uint64_t offset,
  const void *hostBuf, size_t size) {
  std::cout << "xclWrite: " << handle << std::endl;
  return static_cast<size_t>(-EINVAL);
}

size_t xclRead(
  xclDeviceHandle handle, xclAddressSpace space, uint64_t offset,
  void *hostBuf, size_t size) {
  std::cout << "xclRead: " << handle << std::endl;
  return static_cast<size_t>(-EINVAL);
}

int xclGetDeviceInfo2(xclDeviceHandle handle, xclDeviceInfo2 *info) {
  std::cout << "xclRead: " << handle << std::endl;
  return -EINVAL;
}

int xclGetBOProperties(
  xclDeviceHandle handle, unsigned int boHandle, xclBOProperties *properties) {
  std::cout << "xclGetBOProperties: " << handle << std::endl;
  return -EINVAL;
}

unsigned int xclVersion () {
  return 2;
}

int xclExecBuf(xclDeviceHandle handle, unsigned int cmdBO) {
  std::cout << "xclExecBuf: " << handle << std::endl;
  return -EINVAL;
}

int xclExecWait(xclDeviceHandle handle, int timeoutMilliSec) {
  std::cout << "xclExecWait: " << handle << std::endl;
  return -EINVAL;
}

unsigned int xclGetNumLiveProcesses(xclDeviceHandle handle) {
  std::cout << "xclGetNumLiveProcesses: " << handle << std::endl;
  return 0;
}

int xclGetSysfsPath(
  xclDeviceHandle handle, const char* subdev,
  const char* entry, char* sysfsPath, size_t size) {
  std::cout << "xclGetSysfsPath: " << handle << std::endl;
  return -EINVAL;
}

int xclGetDebugIPlayoutPath(xclDeviceHandle handle, char* layoutPath, size_t size) {
  std::cout << "xclGetDebugIPlayoutPath: " << handle << std::endl;
  return -EINVAL;
}

int xclGetTraceBufferInfo(
  xclDeviceHandle handle, uint32_t nSamples, uint32_t& traceSamples, uint32_t& traceBufSz) {
  std::cout << "xclGetTraceBufferInfo: " << handle << std::endl;
  return -EINVAL;
}

int xclReadTraceData(
  xclDeviceHandle handle, void* traceBuf, uint32_t traceBufSz, uint32_t numSamples,
  uint64_t ipBaseAddress, uint32_t& wordsPerSample) {
  std::cout << "xclReadTraceData: " << handle << std::endl;
  return -EINVAL;
}

double xclGetDeviceClockFreqMHz(xclDeviceHandle handle) {
  std::cout << "xclGetDeviceClockFreqMHz: " << handle << std::endl;
  return 0;
}

double xclGetHostReadMaxBandwidthMBps(xclDeviceHandle handle) {
  std::cout << "xclGetHostReadMaxBandwidthMBps: " << handle << std::endl;
  return 0.0;
}


double xclGetHostWriteMaxBandwidthMBps(xclDeviceHandle handle) {
  std::cout << "xclGetHostWriteMaxBandwidthMBps: " << handle << std::endl;
  return 0.0;
}


double xclGetKernelReadMaxBandwidthMBps(xclDeviceHandle handle) {
  std::cout << "xclGetKernelReadMaxBandwidthMBpsl: " << handle << std::endl;
  return 0.0;
}


double xclGetKernelWriteMaxBandwidthMBps(xclDeviceHandle handle) {
  std::cout << "xclGetKernelWriteMaxBandwidthMBps: " << handle << std::endl;
  return 0.0;
}

#if 0
int xclSKGetCmd(xclDeviceHandle handle, xclSKCmd *cmd) {
  std::cout << "xclSKGetCmd: " << handle << std::endl;
  return -EINVAL;
}

int xclAIEGetCmd(xclDeviceHandle handle, xclAIECmd *cmd) {
  std::cout << "xclAIEGetCmd: " << handle << std::endl;
  return -EINVAL;
}

int xclAIEPutCmd(xclDeviceHandle handle, xclAIECmd *cmd) {
  std::cout << "xclAIEPutCmd: " << handle << std::endl;
  return -EINVAL;
}

int xclSKCreate(xclDeviceHandle handle, int *boHandle, uint32_t cu_idx) {
  std::cout << "xclSKCreate: " << handle << std::endl;
  return -EINVAL;
}

int xclSKReport(xclDeviceHandle handle, uint32_t cu_idx, xrt_scu_state state) {
  std::cout << "xclSKReport: " << handle << std::endl;
  return -EINVAL;
}
#endif

/*
 * Context switch phase 1: support xclbin swap, no cu and shared checking
 */
int xclOpenContext(
  xclDeviceHandle handle, const uuid_t xclbinId, unsigned int ipIndex, bool shared) {
  std::cout << "xclOpenContext: " << handle << std::endl;
  return -EINVAL;
}

int xclOpenContextByName(
  xclDeviceHandle handle, uint32_t slot, const uuid_t xclbinId, const char* cuname,
  bool shared) {
  std::cout << "xclOpenContextByName: " << handle << std::endl;
  return -ENOENT;
}

int xclCloseContext(xclDeviceHandle handle, const uuid_t xclbinId, unsigned ipIndex) {
  std::cout << "xclCloseContext: " << handle << std::endl;
  return -EINVAL;
}

size_t xclGetDeviceTimestamp(xclDeviceHandle handle) {
  std::cout << "xclGetDeviceTimestamp: " << handle << std::endl;
  return 0;
}

size_t xclDebugReadIPStatus(
  xclDeviceHandle handle, xclDebugReadType type, void* debugResults) {
  std::cout << "xclDebugReadIPStatus: " << handle << " type=" << type << std::endl;
  switch (type) {
  case XCL_DEBUG_READ_TYPE_LAPC:
    break;
  case XCL_DEBUG_READ_TYPE_AIM:
    break;
  case XCL_DEBUG_READ_TYPE_AM:
    break;
  case XCL_DEBUG_READ_TYPE_ASM:
    break;
  case XCL_DEBUG_READ_TYPE_SPC:
    break;
  default:
    break;
  }
  return size_t(-1);
}

int xclGetDebugProfileDeviceInfo(xclDeviceHandle handle, xclDebugProfileDeviceInfo* info) {
  std::cout << "xclGetDebugProfileDeviceInfo: " << handle << std::endl;
  return 0;
}

int XXXX_xclResetDevice(xclDeviceHandle handle, xclResetKind kind) {
  std::cout << "_xclResetDevice: " << handle << std::endl;
  return 0;
}

int xclGetUsageInfo(xclDeviceHandle handle, xclDeviceUsage *info) {
  std::cout << "xclGetUsageInfo: " << handle << std::endl;
  return 0;
}

int xclGetErrorStatus(xclDeviceHandle handle, xclErrorStatus *info) {
  std::cout << "xclGetErrorStatus: " << handle << std::endl;
  return 0;
}

int xclReClock2(xclDeviceHandle handle, unsigned short region, const unsigned short *targetFreqMHz) {
  std::cout << "xclDeviceHandle: " << handle << std::endl;
  return 0;
}

int xclLockDevice(xclDeviceHandle handle) {
  std::cout << "xclDeviceHandle: " << handle << std::endl;
  return 0;
}

int xclUnlockDevice(xclDeviceHandle handle) {
  std::cout << "xclUnlockDevice: " << handle << std::endl;
  return 0;
}

int xclUpgradeFirmware(xclDeviceHandle handle, const char *fileName) {
  std::cout << "xclUpgradeFirmware: " << handle << std::endl;
  return 0;
}

int xclUpgradeFirmware2(xclDeviceHandle handle, const char *file1, const char* file2) {
  std::cout << "xclUpgradeFirmware2: " << handle << std::endl;
  return 0;
}

int xclUpgradeFirmwareXSpi(xclDeviceHandle handle, const char *fileName, int index) {
  std::cout << "xclUpgradeFirmwareXSpi: " << handle << std::endl;
  return 0;
}

int xclBootFPGA(xclDeviceHandle handle) {
  std::cout << "xclBootFPGA: " << handle << std::endl;
  return 0;
}

int xclRemoveAndScanFPGA() {
  std::cout << "xclRemoveAndScanFPGA: " << std::endl;
  return 0;
}

ssize_t xclUnmgdPread(
  xclDeviceHandle handle, unsigned flags, void *buf, size_t count, uint64_t offset){
  std::cout << "xclUnmgdPread: " << handle << std::endl;
  return -ENOSYS;
}

ssize_t xclUnmgdPwrite(
  xclDeviceHandle handle, unsigned flags, const void *buf, size_t count, uint64_t offset) {
  std::cout << "xclUnmgdPwrite: " << handle << std::endl;
  return -ENOSYS;
}

int xclRegisterInterruptNotify(xclDeviceHandle handle, unsigned int userInterrupt, int fd) {
  std::cout << "xclRegisterInterruptNotify: " << handle << std::endl;
  return 0;
}

int xclCreateWriteQueue(xclDeviceHandle handle, xclQueueContext *q_ctx, void **q_hdl) {
  std::cout << "xclCreateWriteQueue: " << handle << std::endl;
  return -ENOSYS;
}

int xclCreateReadQueue(xclDeviceHandle handle, xclQueueContext *q_ctx, void **q_hdl) {
  std::cout << "xclCreateReadQueue: " << handle << std::endl;
  return -ENOSYS;
}

int xclDestroyQueue(xclDeviceHandle handle, void *q_hdl) {
  std::cout << "xclDestroyQueue: " << handle << std::endl;
  return -ENOSYS;
}

int xclModifyQueue(xclDeviceHandle handle, void *q_hdl) {
  std::cout << "xclModifyQueue: " << handle << std::endl;
  return -ENOSYS;
}

int xclStartQueue(xclDeviceHandle handle, void *q_hdl) {
  std::cout << "xclStartQueue: " << handle << std::endl;
  return -ENOSYS;
}

int xclStopQueue(xclDeviceHandle handle, void *q_hdl) {
  std::cout << "xclStopQueue: " << handle << std::endl;
  return -ENOSYS;
}

ssize_t xclWriteQueue(xclDeviceHandle handle, void *q_hdl, xclQueueRequest *wr_req) {
  std::cout << "xclWriteQueue: " << handle << std::endl;
  return -ENOSYS;
}

ssize_t xclReadQueue(xclDeviceHandle handle, void *q_hdl, xclQueueRequest *wr_req) {
  std::cout << "xclReadQueue: " << handle << std::endl;
  return -ENOSYS;
}

int xclCreateProfileResults(xclDeviceHandle handle, ProfileResults** results) {
  std::cout << "xclCreateProfileResults: " << handle << std::endl;
  int status = -1;
  return -ENODEV;
}

int xclGetProfileResults(xclDeviceHandle handle, ProfileResults* results) {
  std::cout << "xclGetProfileResults: " << handle << std::endl;
  int status = -1;
  return -ENODEV;
}

int xclDestroyProfileResults(xclDeviceHandle handle, ProfileResults* results) {
  std::cout << "xclDestroyProfileResults: " << handle << std::endl;
  int status = -1;
  return -ENODEV;
}

int xclRegWrite(
  xclDeviceHandle handle, uint32_t ipIndex, uint32_t offset, uint32_t data) {
  std::cout << "xclRegWrite: " << handle << std::endl;
  return -ENODEV;
}

int xclRegRead(
  xclDeviceHandle handle, uint32_t ipIndex, uint32_t offset, uint32_t *datap) {
  std::cout << "xclRegRead: " << handle << std::endl;
  return -ENODEV;
}

int xclIPName2Index(xclDeviceHandle handle, const char *name) {
  std::cout << "xclIPName2Index: " << handle << std::endl;
  return -ENOENT;
}

int xclLogMsg(
  xclDeviceHandle handle, xrtLogMsgLevel level, const char* tag, const char* format, ...) {
  std::cout << "xclLogMsg: " << handle << std::endl;
  static auto verbosity = xrt_core::config::get_verbosity();
  if (level > verbosity)
    return 0;

  va_list args;
  va_start(args, format);
  xrt_core::message::sendv(static_cast<xrt_core::message::severity_level>(level), tag, format, args);
  va_end(args);

  return 0;
}

int xclOpenIPInterruptNotify(
  xclDeviceHandle handle, uint32_t ipIndex, unsigned int flags) {
  std::cout << "xclOpenIPInterruptNotify: " << handle << std::endl;
  return -EINVAL;
}

int xclCloseIPInterruptNotify(xclDeviceHandle handle, int fd) {
  std::cout << "xclCloseIPInterruptNotify: " << handle << std::endl;
  return -EINVAL;
}

void xclGetDebugIpLayout(
  xclDeviceHandle handle, char* buffer, size_t size, size_t* size_ret) {
  std::cout << "xclGetDebugIpLayout: " << handle << std::endl;
  if(size_ret)
    *size_ret = 0;
  return;
}

int xclGetSubdevPath(
  xclDeviceHandle handle,  const char* subdev, uint32_t idx, char* path,
  size_t size) {
  std::cout << "xclGetSubdevPath: " << handle << std::endl;
  return 0;
}

int
xclP2pEnable(xclDeviceHandle handle, bool enable, bool force) {
  std::cout << "xclP2pEnable: " << handle << std::endl;
  return 1; // -ENOSYS;
}

int
xclCmaEnable(xclDeviceHandle handle, bool enable, uint64_t force)
{
  std::cout << "xclCmaEnable: " << handle << std::endl;
  return -ENOSYS;
}

int xclUpdateSchedulerStat(xclDeviceHandle handle) {
  std::cout << "xclUpdateSchedulerStat: " << handle << std::endl;
  return 1; // -ENOSYS;
}

int xclResetDevice(xclDeviceHandle handle, xclResetKind kind) {
  std::cout << "xclResetDevice: " << handle << std::endl;
  return -ENOSYS;
}

int xclInternalResetDevice(xclDeviceHandle handle, xclResetKind kind) {
  std::cout << "xclInternalResetDevice: " << handle << std::endl;
  return -ENOSYS;
}

int xclErrorInject(
  xclDeviceHandle handle, uint16_t num, uint16_t driver, uint16_t severity,
  uint16_t module, uint16_t eclass) {
  std::cout << "xclErrorInject: " << handle << std::endl;
  return -EINVAL;
}

int xclErrorClear(xclDeviceHandle handle) {
  std::cout << "xclErrorClear: " << handle << std::endl;
  return -EINVAL;
}
#endif
