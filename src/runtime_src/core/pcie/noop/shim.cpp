// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2021-2022 Xilinx, Inc. All rights reserved.

// This file implements a dummy (no-op) shim level driver that is
// used exclusively for debugging user space XRT with HW xclbins
//
// The code is not beautified in any way or fashion, it is meant
// for quick and dirty validation of user space code changes.
#define XCL_DRIVER_DLL_EXPORT
#define XRT_CORE_PCIE_NOOP_SOURCE
#include "shim.h"                  // This file implements shim.h
#include "core/include/shim_int.h" // This file implements shim_int.h
#include "core/include/ert.h"

#include "core/common/config_reader.h"
#include "core/common/device.h"
#include "core/common/message.h"
#include "core/common/system.h"
#include "core/common/task.h"
#include "core/common/thread.h"

#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <string>

namespace { // private implementation details

using device_index_type = unsigned int;

namespace buffer {

struct bo
{
  void* hbuf = nullptr;
  void* own = nullptr;
  void* dbuf = reinterpret_cast<void*>(0xdeadbeef);
  size_t size = 0;
  unsigned int flags = 0;

  bo(void* uptr, size_t bytes, unsigned int flgs)
    : hbuf(uptr), size(bytes), flags(flgs)
  {}

  bo(size_t bytes, unsigned int flgs)
    : hbuf(malloc(bytes)), own(hbuf), size(bytes), flags(flgs)
  {
    std::memset(hbuf, 0, size);
  }

  ~bo()
  { if (own) free(own); }
};

static std::mutex mutex;
static unsigned int handle = 0;
static std::map<unsigned int, std::unique_ptr<bo>> h2b;

auto
find(unsigned int handle)
{
  std::lock_guard<std::mutex> lk(mutex);
  auto itr = h2b.find(handle);
  if (itr == h2b.end())
    throw std::runtime_error("no such bo handle: " + std::to_string(handle));
  return itr;
}

bo*
get(unsigned int handle)
{
  auto itr = find(handle);
  return (*itr).second.get();
}

unsigned int
alloc(size_t size, unsigned int flags)
{
  std::lock_guard<std::mutex> lk(mutex);
  h2b.insert(std::make_pair(handle, std::make_unique<bo>(size, flags)));
  return handle++;
}

unsigned int
alloc(void* uptr, size_t size, unsigned int flags)
{
  std::lock_guard<std::mutex> lk(mutex);
  h2b.insert(std::make_pair(handle, std::make_unique<bo>(uptr, size, flags)));
  return handle++;
}

void*
map(unsigned int handle)
{
  auto bo = get(handle);
  return bo->hbuf;
}

void
free(unsigned int handle)
{
  auto itr = find(handle);
  h2b.erase(itr);
}

} // buffer

// Model the programming part of the device.
// Pretend a device that supports multiple xclbins, which can can be
// loaded by host application into different slots, where slots are
// chosen by the driver (this code).
namespace pl {

class device
{
  // model multiple partitions
  using slot_id = xrt_core::device::slot_id;
  std::map<slot_id, xrt::xclbin> xclbins;

  // capture cu data based on which slot it is associated with
  struct cu_data {
    std::string name;  // cu name
    slot_id slot = 0;  // slot in which this cu is opened
    uint32_t ctx = 0;  // how many contexts are opened on the cu
  };
  std::map<uint32_t, cu_data> idx2cu;  // idx -> cu_data

  // cu indices are per device from 0..127.
  // keep a stack of indices that can be used
  // once last context on cu is released, its index is recycled
  static constexpr uint32_t cu_max = 128;
  std::vector<uint32_t> free_cu_indices;  // push, back, pop

  // slot index for xclbin is a running incremented index
  slot_id slot_index = 0; // running index

  // exclusive locking to prevent race
  std::mutex mutex;

  // assign a slot index to an xclbin
  void
  register_xclbin(slot_id slot, const xrt::xclbin& xclbin)
  {
    xclbins[slot] = xclbin;
  }

public:
  // device ctor, initialize free cu indices
  device()
  {
    free_cu_indices.reserve(128);
    for (int i = 127; i >=0; --i)
      free_cu_indices.push_back(i);
  }

  // assign next slot index to an xclbin
  void
  register_xclbin(const xrt::xclbin& xclbin)
  {
    std::lock_guard lk(mutex);
    register_xclbin(slot_index++, xclbin);
  }

  // unload an xclbin (todo)
  void
  unload_xclbin(slot_id slot)
  {
    std::lock_guard lk(mutex);
    xclbins.erase(slot);
  }

  // open context on cu
  // return: 0 on success, EAGAIN if xclbin was re-loaded, else error
  int
  open_context(slot_id slot, const xrt::uuid& xid, const std::string& cuname, bool shared)
  {
    std::lock_guard lk(mutex);

    // this xclbin must have been registered
    const auto& xclbin = xclbins[slot];
    if (!xclbin)
      throw xrt_core::error("Slot xclbin mismatch, no such registered xclbin in slot: " + std::to_string(slot));

    // find cu in xclbin
    auto cu = xclbin.get_ip(cuname);
    if (!cu)
      throw xrt_core::error("No such cu: " + cuname);

    // Current user-level implementation only attempts opening of context
    // once per CU.  In other words, it is an error if this function is
    // called twice on same CU within same process and since this noop driver
    // is tied to a process we simply throw.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
    for (const auto& [idx, cud] : idx2cu)
      if (cud.name == cuname && cud.slot == slot)
        throw xrt_core::error("Context already opened on cu: " + cuname);
#pragma GCC diagnostic pop

#if 0 // enable to pretend to fail and re-load xclbin to different additional slot
    if (slot == 0) {
      register_xclbin(slot_index++, xclbin);
      return EAGAIN;
    }
#endif

    auto idx = free_cu_indices.back();
    auto& cudata = idx2cu[idx];
    cudata.name = cuname;
    cudata.slot = slot;
    cudata.ctx = 1;
    free_cu_indices.pop_back();

    return 0;
  }

  // close the cu context
  void
  close_context(const xrt::uuid& xid, uint32_t cuidx)
  {
    std::lock_guard lk(mutex);

    auto cu_itr = idx2cu.find(cuidx);
    if (cu_itr == idx2cu.end())
      throw xrt_core::error("No such cu with index: " + std::to_string(cuidx));
    auto& cudata  = (*cu_itr).second;
    if (cudata.ctx < 1)
      throw xrt_core::error("No context aquired on cu: " + std::to_string(cuidx));

    // clean up if last context was released
    if (!--cudata.ctx) {
      auto nm = cudata.name;
      idx2cu.erase(cuidx);
      free_cu_indices.push_back(cuidx);
    }
  }

  xrt_core::query::kds_cu_info::result_type
  kds_cu_info()
  {
    xrt_core::query::kds_cu_info::result_type vec;
    for (const auto& [idx, cud] : idx2cu) {
      xrt_core::query::kds_cu_info::data data;
      data.slot_index = cud.slot;
      data.index = idx;
      data.name = cud.name;
      data.base_addr = 0xdeadbeef;
      data.status = 0;
      data.usages = 0;
      vec.push_back(std::move(data));
    }
    return vec;
  }

  xrt_core::query::xclbin_slots::result_type
  xclbin_slots()
  {
    xrt_core::query::xclbin_slots::result_type vec;
    for (const auto& [slot, xclbin] : xclbins) {
      xrt_core::query::xclbin_slots::slot_info data;
      data.slot = slot;
      data.uuid = xclbin.get_uuid().to_string();
      vec.push_back(std::move(data));
    }
    return vec;
  }

};

} // pl

static std::vector<std::shared_ptr<pl::device>> s_devices;


// Simulate asynchronous command completion.
//
// Command handles are added to a producer/consumer queue A worker
// thread pretends to run the command and marks it complete only if
// the command was enqueue some constant time before now.
namespace cmd {

static unsigned int completion_delay_us = 0;
static xrt_core::task::queue running_queue;
static std::thread completer;
static std::atomic<uint64_t> completion_count {0};

struct cmd_type
{
  xclBufferHandle handle;
  unsigned long queue_time;
  cmd_type(xclBufferHandle h)
    : handle(h), queue_time(xrt_core::time_ns())
  {}
};

static void
init()
{
  if ( (completion_delay_us = xrt_core::config::get_noop_completion_delay_us()) )
    completer = std::move(xrt_core::thread(xrt_core::task::worker, std::ref(running_queue)));
}

static void
stop()
{
  if (completion_delay_us) {
    running_queue.stop();
    completer.join();
  }
}

static void
wait()
{
  while (!completion_count) ;
  --completion_count;
}

static void
mark_cmd_handle_complete(xclBufferHandle handle)
{
  //XRT_PRINTF("handle(%d) is complete\n", handle);
  auto hbuf = buffer::map(handle);
  auto cmd = reinterpret_cast<ert_packet*>(hbuf);
  cmd->state = ERT_CMD_STATE_COMPLETED;
  ++completion_count;
}

static void
mark_cmd_complete(cmd_type ct)
{
  while (xrt_core::time_ns() - ct.queue_time < completion_delay_us * 1000);
  mark_cmd_handle_complete(ct.handle);
}

static void
add(xclBufferHandle handle)
{
  if (completion_delay_us)
    xrt_core::task::createF(running_queue, mark_cmd_complete, cmd_type(handle));
  else
    mark_cmd_handle_complete(handle);
}

struct X
{
  X() { init(); }
  ~X() { stop(); }
};

static X x;

} // cmd


struct shim
{
  using buffer_handle_type = xclBufferHandle; // xrt.h
  unsigned int m_devidx;
  bool m_locked = false;
  pl::device* m_pldev;
  std::shared_ptr<xrt_core::device> m_core_device;

  // create shim object, open the device, store the device handle
  shim(unsigned int devidx)
    : m_devidx(devidx), m_core_device(xrt_core::get_userpf_device(this, m_devidx))
  {
    if (s_devices.size() <= devidx)
      s_devices.resize(devidx + 1);

    if (!s_devices[devidx])
      s_devices[devidx] = std::make_shared<pl::device>();
    m_pldev = s_devices[devidx].get();
  }

  // destruct shim object, close the device
  ~shim()
  {}

  buffer_handle_type
  alloc_bo(size_t size, unsigned int flags)
  {
    return buffer::alloc(size, flags);
  }

  buffer_handle_type
  alloc_user_ptr_bo(void* userptr, size_t size, unsigned int flags)
  {
    return buffer::alloc(userptr, size, flags);
  }

  void*
  map_bo(buffer_handle_type handle, bool /*write*/)
  {
    return buffer::map(handle);
  }

  int
  unmap_bo(buffer_handle_type handle, void* addr)
  {
    return 0;
  }

  void
  free_bo(buffer_handle_type handle)
  {
    buffer::free(handle);
  }

  int
  sync_bo(buffer_handle_type, xclBOSyncDirection, size_t, size_t)
  {
    return 0;
  }

  int
  open_context(const xrt::uuid&, unsigned int, bool)
  {
    return 0;
  }

  int
  open_context(uint32_t slot, const xrt::uuid& uuid, const char* cuname, bool shared)
  {
    return m_pldev->open_context(slot, uuid, cuname, shared);
  }

  int
  close_context(const xrt::uuid& xid, unsigned int idx)
  {
    m_pldev->close_context(xid, idx);
    return 0;
  }

  int
  exec_buf(buffer_handle_type handle)
  {
    cmd::add(handle);
    return 0;
  }

  int
  exec_wait(int msec)
  {
    cmd::wait();
    return 1;
  }

  int
  get_bo_properties(buffer_handle_type handle, struct xclBOProperties* properties)
  {
    auto bo = buffer::get(handle);
    properties->handle = handle;
    properties->flags = bo->flags;
    properties->size = bo->size;
    properties->paddr = reinterpret_cast<uint64_t>(bo->dbuf);

    return 0;
  }

  int
  load_xclbin(const struct axlf* top)
  {
    auto xclbin = m_core_device->get_xclbin(top->m_header.uuid);
    m_pldev->register_xclbin(xclbin);
    return 0;
  }

  int
  write(enum xclAddressSpace, uint64_t, const void*, size_t)
  {
    throw std::runtime_error("not implemented");
  }

  int
  read(enum xclAddressSpace, uint64_t, void*, size_t)
  {
    throw std::runtime_error("not implemented");
  }

  ssize_t
  unmgd_pwrite(unsigned int, const void*, size_t, uint64_t)
  {
    throw std::runtime_error("not implemented");
  }

  ssize_t
  unmgd_pread(unsigned int, void*, size_t, uint64_t)
  {
    throw std::runtime_error("not implemented");
  }

  int
  write_bo(xclBufferHandle handle, const void *src, size_t size, size_t seek)
  {
    auto bo = buffer::get(handle);
    auto hbuf = reinterpret_cast<char*>(bo->hbuf) + seek;
    std::memcpy(hbuf, src, std::min(bo->size - seek, size));
    return 0;
  }

  int
  read_bo(xclBufferHandle handle, void *dst, size_t size, size_t skip)
  {
    auto bo = buffer::get(handle);
    auto hbuf = reinterpret_cast<char*>(bo->hbuf) + skip;
    std::memcpy(dst, hbuf, std::min(bo->size - skip, size));
    return 0;
  }

}; // struct shim

shim*
get_shim_object(xclDeviceHandle handle)
{
  // TODO: Do some sanity check
  return reinterpret_cast<shim*>(handle);
}

} // namespace

namespace userpf {

xrt_core::query::kds_cu_info::result_type
kds_cu_info(const xrt_core::device* device)
{
  auto id = device->get_device_id();
  if (s_devices.size() < id)
    throw xrt_core::error("Unknown device id: " + std::to_string(id));

  xrt_core::query::kds_cu_info::result_type vec;
  return s_devices[id]->kds_cu_info();
}

xrt_core::query::xclbin_slots::result_type
xclbin_slots(const xrt_core::device* device)
{
  auto id = device->get_device_id();
  if (s_devices.size() < id)
    throw xrt_core::error("Unknown device id: " + std::to_string(id));

  xrt_core::query::xclbin_slots::result_type vec;
  return s_devices[id]->xclbin_slots();
}

}

// Basic
unsigned int
xclProbe()
{
  return 1;
}

xclDeviceHandle
xclOpen(unsigned int deviceIndex, const char *logFileName, xclVerbosityLevel level)
{
  try {
    xrt_core::message::
      send(xrt_core::message::severity_level::debug, "XRT", "xclOpen()");
    return new shim(deviceIndex);
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
  }
  catch (const std::exception& ex) {
    xrt_core::send_exception_message(ex.what());
  }

  return nullptr;
}

void
xclClose(xclDeviceHandle handle)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclClose()");
  auto shim = get_shim_object(handle);
  delete shim;
}


// XRT Buffer Management APIs
xclBufferHandle
xclAllocBO(xclDeviceHandle handle, size_t size, int unused, unsigned int flags)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclAllocBO()");
  auto shim = get_shim_object(handle);
  return shim->alloc_bo(size, flags);
}

xclBufferHandle
xclAllocUserPtrBO(xclDeviceHandle handle, void *userptr, size_t size, unsigned int flags)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclAllocUserPtrBO()");
  auto shim = get_shim_object(handle);
  return shim->alloc_user_ptr_bo(userptr, size, flags);
}

void*
xclMapBO(xclDeviceHandle handle, xclBufferHandle boHandle, bool write)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclMapBO()");
  auto shim = get_shim_object(handle);
  return shim->map_bo(boHandle, write);
}

int
xclUnmapBO(xclDeviceHandle handle, xclBufferHandle boHandle, void* addr)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclUnmapBO()");
  auto shim = get_shim_object(handle);
  return shim->unmap_bo(boHandle, addr);
}

void
xclFreeBO(xclDeviceHandle handle, xclBufferHandle boHandle)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclFreeBO()");
  auto shim = get_shim_object(handle);
  return shim->free_bo(boHandle);
}

int
xclSyncBO(xclDeviceHandle handle, xclBufferHandle boHandle, xclBOSyncDirection dir, size_t size, size_t offset)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclSyncBO()");
  auto shim = get_shim_object(handle);
  return shim->sync_bo(boHandle, dir, size, offset);
}

int
xclCopyBO(xclDeviceHandle handle, xclBufferHandle dstBoHandle,
          xclBufferHandle srcBoHandle, size_t size, size_t dst_offset,
          size_t src_offset)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclCopyBO() NOT IMPLEMENTED");
  return ENOSYS;
}

int
xclReClock2(xclDeviceHandle handle, unsigned short region,
            const uint16_t* targetFreqMHz)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclReClock2() NOT IMPLEMENTED");
  return ENOSYS;
}

// Compute Unit Execution Management APIs
int
xclOpenContext(xclDeviceHandle handle, const xuid_t xclbinId, unsigned int ipIndex, bool shared)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclOpenContext()");
  auto shim = get_shim_object(handle);

  // Virtual resources are not currently supported by driver
  return (ipIndex == (unsigned int)-1)
    ? 0
    : shim->open_context(xclbinId, ipIndex, shared);
}

int
xclOpenContextByName(xclDeviceHandle handle, uint32_t slot, const xuid_t xclbin_uuid, const char* cuname, bool shared)
{
  try {
    auto shim = get_shim_object(handle);
    return shim->open_context(slot, xclbin_uuid, cuname, shared);
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
    return ex.get_code();
  }
  catch (const std::exception& ex) {
    xrt_core::send_exception_message(ex.what());
    return -ENOENT;
  }
}

int
xclCloseContext(xclDeviceHandle handle, const xuid_t xclbinId, unsigned int ipIndex)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclCloseContext()");
  auto shim = get_shim_object(handle);

  try {
    // Virtual resources are not currently supported by driver
    return (ipIndex == (unsigned int) -1) ? 0 : shim->close_context(xclbinId, ipIndex);
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
    return ex.get_code();
  }
  catch (const std::exception& ex) {
    xrt_core::send_exception_message(ex.what());
    return -ENOENT;
  }
}

int
xclExecBuf(xclDeviceHandle handle, xclBufferHandle cmdBO)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclExecBuf()");
  auto shim = get_shim_object(handle);
  return shim->exec_buf(cmdBO);
}

int
xclExecWait(xclDeviceHandle handle, int timeoutMilliSec)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclExecWait()");
  auto shim = get_shim_object(handle);
  return shim->exec_wait(timeoutMilliSec);
}

xclBufferExportHandle
xclExportBO(xclDeviceHandle handle, xclBufferHandle boHandle)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclExportBO() NOT IMPLEMENTED");
  return XRT_NULL_BO_EXPORT;
}

xclBufferHandle
xclImportBO(xclDeviceHandle handle, xclBufferExportHandle fd, unsigned flags)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclImportBO() NOT IMPLEMENTED");
  return XRT_NULL_BO_EXPORT;
}

int
xclCloseExportHandle(xclBufferExportHandle fd)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclCloseExportHandle() NOT IMPLEMENTED");
  return 0;
}

int
xclGetBOProperties(xclDeviceHandle handle, xclBufferHandle boHandle,
		   struct xclBOProperties *properties)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclGetBOProperties()");
  auto shim = get_shim_object(handle);
  return shim->get_bo_properties(boHandle,properties);
}

int
xclLoadXclBin(xclDeviceHandle handle, const struct axlf *buffer)
{
  try {
    xrt_core::message::
      send(xrt_core::message::severity_level::debug, "XRT", "xclLoadXclbin()");
    auto shim = get_shim_object(handle);
    if (auto ret = shim->load_xclbin(buffer))
      return ret;
    auto core_device = xrt_core::get_userpf_device(shim);
    core_device->register_axlf(buffer);
    return 0;
  }
  catch (const xrt_core::error& ex) {
    xrt_core::send_exception_message(ex.what());
    return ex.get_code();
  }
  catch (const std::exception& ex) {
    xrt_core::send_exception_message(ex.what());
    return -EINVAL;
  }
}

unsigned int
xclVersion()
{
  return 2;
}

int
xclGetDeviceInfo2(xclDeviceHandle handle, struct xclDeviceInfo2 *info)
{
  std::memset(info, 0, sizeof(xclDeviceInfo2));
  info->mMagic = 0;
  info->mHALMajorVersion = XCLHAL_MAJOR_VER;
  info->mHALMinorVersion = XCLHAL_MINOR_VER;
  info->mMinTransferSize = 0;
  info->mDMAThreads = 2;
  info->mDataAlignment = 4096; // 4k

  return 0;
}

int
xclLockDevice(xclDeviceHandle handle)
{
  return 0;
}

int
xclUnlockDevice(xclDeviceHandle handle)
{
  return 0;
}

ssize_t
xclUnmgdPwrite(xclDeviceHandle handle, unsigned int flags, const void *buf, size_t count, uint64_t offset)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclUnmgdPwrite()");
  auto shim = get_shim_object(handle);
  return shim->unmgd_pwrite(flags, buf, count, offset) ? 0 : 1;
}

ssize_t
xclUnmgdPread(xclDeviceHandle handle, unsigned int flags, void *buf, size_t count, uint64_t offset)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclUnmgdPread()");
  auto shim = get_shim_object(handle);
  return shim->unmgd_pread(flags, buf, count, offset) ? 0 : 1;
}

size_t xclWriteBO(xclDeviceHandle handle, xclBufferHandle boHandle, const void *src, size_t size, size_t seek)
{
    xrt_core::message::
        send(xrt_core::message::severity_level::debug, "XRT", "xclWriteBO()");
    auto shim = get_shim_object(handle);
    return shim->write_bo(boHandle, src, size, seek);
}

size_t xclReadBO(xclDeviceHandle handle, xclBufferHandle boHandle, void *dst, size_t size, size_t skip)
{
    xrt_core::message::
        send(xrt_core::message::severity_level::debug, "XRT", "xclReadBO()");
    auto shim = get_shim_object(handle);
    return shim->read_bo(boHandle, dst, size, skip);
}

void
xclGetDebugIpLayout(xclDeviceHandle hdl, char* buffer, size_t size, size_t* size_ret)
{
}

// Deprecated APIs
size_t
xclWrite(xclDeviceHandle handle, enum xclAddressSpace space, uint64_t offset, const void *hostbuf, size_t size)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclWrite()");
  auto shim = get_shim_object(handle);
  return shim->write(space,offset,hostbuf,size) ? 0 : size;
}

size_t
xclRead(xclDeviceHandle handle, enum xclAddressSpace space,
        uint64_t offset, void *hostbuf, size_t size)
{
  xrt_core::message::
    send(xrt_core::message::severity_level::debug, "XRT", "xclRead()");
  auto shim = get_shim_object(handle);
  return shim->read(space,offset,hostbuf,size) ? 0 : size;
}

// Restricted read/write on IP register space
int
xclRegWrite(xclDeviceHandle handle, uint32_t ipidx, uint32_t offset, uint32_t data)
{
  return 1;
}

int
xclRegRead(xclDeviceHandle handle, uint32_t ipidx, uint32_t offset, uint32_t* datap)
{
  return 1;
}

int
xclGetTraceBufferInfo(xclDeviceHandle handle, uint32_t nSamples,
                      uint32_t& traceSamples, uint32_t& traceBufSz)
{
  return 0;
}

int
xclReadTraceData(xclDeviceHandle handle, void* traceBuf, uint32_t traceBufSz,
                 uint32_t numSamples, uint64_t ipBaseAddress,
                 uint32_t& wordsPerSample)
{
  return 0;
}

int
xclGetSubdevPath(xclDeviceHandle handle,  const char* subdev,
                 uint32_t idx, char* path, size_t size)
{
  return 0;
}

int
xclP2pEnable(xclDeviceHandle handle, bool enable, bool force)
{
  return 1; // -ENOSYS;
}

int
xclUpdateSchedulerStat(xclDeviceHandle handle)
{
  return 1; // -ENOSYS;
}

int
xclCmaEnable(xclDeviceHandle handle, bool enable, uint64_t force)
{
  return -ENOSYS;
}

int
xclInternalResetDevice(xclDeviceHandle handle, xclResetKind kind)
{
  return 1; // -ENOSYS;
}
