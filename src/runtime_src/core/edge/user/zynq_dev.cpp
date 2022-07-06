/**
 * Copyright (C) 2019-2022 Xilinx, Inc
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

#include <fstream>
#include <iostream>
#include <sstream>
#include <cstring>
#include <boost/filesystem.hpp>
#include <regex>
#include "zynq_dev.h"

/*
 * POSIX open/close are only in unistd.h and this is not
 * not in the C++ std namespace.
 */
#include <fcntl.h>
#include <unistd.h>

#include "core/edge/include/zynq_ioctl.h"

/*
 * Provide a means to handle the sysfs so hosts that do not have sysfs
 * can provide suitable support. Both methods are POSIX OK so we can
 * leave both enabled and select at the last moment. This way the
 * compile gets to check everything on every build.
 *
 * This code assumes only Linux provides sysfs support.
 */
#
struct dev_handle_base {
    const std::string& root;

    dev_handle_base(const std::string& root);
    virtual ~dev_handle_base();

    virtual std::string get_path(const std::string& entry) = 0;

    virtual void open(
        const std::string& entry, std::string& err,
        bool write, bool binary) = 0;

    virtual void get(std::vector<char>& buf) = 0;
    virtual void get(std::vector<std::string>& sv) = 0;
};

dev_handle_base::dev_handle_base(const std::string& root_)
    : root(root_) {
}

dev_handle_base::~dev_handle_base() {
}

struct dev_handle_sysfs : public dev_handle_base {
    std::fstream fs;

    dev_handle_sysfs(const std::string& root);
    ~dev_handle_sysfs() = default;

    std::string get_path(const std::string& entry) override;

    void open(
        const std::string& entry, std::string& err,
        bool write, bool binary) override;

    void get(std::vector<char>& buf) override;
    void get(std::vector<std::string>& sv) override;
};

dev_handle_sysfs::dev_handle_sysfs(const std::string& root_)
    : dev_handle_base(root_) {
}

void dev_handle_sysfs::open(
    const std::string& entry, std::string& err, bool write, bool binary) {
    const std::string path = get_path(entry);
    std::ios::openmode mode = write ? std::ios::out : std::ios::in;

    if (binary)
        mode |= std::ios::binary;

    err.clear();
    fs.open(path, mode);
    if (!fs.is_open()) {
        std::stringstream ss;
        ss << "Failed to open " << path << " for "
            << (binary ? "binary " : "")
            << (write ? "writing" : "reading") << ": "
            << strerror(errno) << std::endl;
        err = ss.str();
    }
}

std::string dev_handle_sysfs::get_path(const std::string& entry) {
    return root + entry;
}

void dev_handle_sysfs::get(std::vector<char>& buf) {
    buf.insert(
        std::end(buf), std::istreambuf_iterator<char>(fs),
        std::istreambuf_iterator<char>());
}

void dev_handle_sysfs::get(std::vector<std::string>& sv) {
    sv.clear();
    std::string line;
    while (std::getline(fs, line))
        sv.push_back(line);
}

struct dev_handle_ioctl : public dev_handle_base {
    int fd;
    std::string entry;
    std::vector<char> data;

    dev_handle_ioctl(const std::string& root);
    virtual ~dev_handle_ioctl() override;

    std::string get_path(const std::string& entry) override;

    void open(
        const std::string& entry, std::string& err,
        bool write, bool binary) override;

    void get(std::vector<char>& buf) override;
    void get(std::vector<std::string>& sv) override;
};

dev_handle_ioctl::dev_handle_ioctl(const std::string& root_)
    : dev_handle_base(root_), fd(-1), data(16 * 1024) {
}

dev_handle_ioctl::~dev_handle_ioctl() {
    std::cout << "]] closing handle: " << fd << std::endl;
    if (fd >= 0) {
        ::close(fd);
    }
}

void dev_handle_ioctl::open(
    const std::string& entry_, std::string& err, bool write, bool binary) {
    const int flags = write ? O_RDWR : O_RDONLY;

    err.clear();

    std::cout << "]] dev_handle_ioctl::open: path=" << root
              << " entry=" << entry_ << std::endl;

    entry = entry_;

    fd = ::open(root.c_str(), flags);
    if (fd < 0) {
        std::stringstream ss;
        ss << "Failed to open " << root << " for "
           << (binary ? "binary " : "")
           << (write ? "writing" : "reading") << ": "
           << strerror(errno) << std::endl;
        err = ss.str();
    }
}

std::string dev_handle_ioctl::get_path(const std::string& entry) {
    return root;
}

void dev_handle_ioctl::get(std::vector<char>& buf) {
}

void dev_handle_ioctl::get(std::vector<std::string>& sv) {
    sv.clear();
    struct drm_zocl_request req = {
        .req_flags = ZOCL_REQ_READ,
        .data_size = uint32_t(data.size()),
        .data_level = 0,
        .data = data.data()
    };
    std::strncpy(req.req_type, entry.c_str(), sizeof(req.req_type) - 1);
    int r = ::ioctl(fd, DRM_IOCTL_ZOCL_REQUEST, &req);
    if (r == 0) {
        std::istringstream iss(data.data());
        std::string line;
        while (std::getline(iss, line)) {
            std::cout << "[] " << line << std::endl;
            sv.push_back(line);
        }
    }
}

/*
 * The sysfs file system and driver is bespoke to Linux.
 *
 * Use ioctl for as a fallback option for those operating systems that
 * do not offer a sysfs type equivalent.
 */
#if __linux__
using dev_handle = dev_handle_sysfs;
#else
using dev_handle = dev_handle_ioctl;
#endif

static std::fstream sysfs_open(const std::string& path, std::string& err,
    bool write, bool binary)
{
    std::fstream fs;
    std::ios::openmode mode = write ? std::ios::out : std::ios::in;

    if (binary)
        mode |= std::ios::binary;

    err.clear();
    fs.open(path, mode);
    if (!fs.is_open()) {
        std::stringstream ss;
        ss << "Failed to open " << path << " for "
            << (binary ? "binary " : "")
            << (write ? "writing" : "reading") << ": "
            << strerror(errno) << std::endl;
        err = ss.str();
    }
    return fs;
}

std::string zynq_device::get_sysfs_path(const std::string& entry)
{
    return sysfs_root + entry;
}

void zynq_device::sysfs_put(const std::string& entry, std::string& err_msg,
    const std::string& input)
{
    std::fstream fs = sysfs_open(get_sysfs_path(entry), err_msg, true, false);
    if (!err_msg.empty())
        return;
    fs << input;
}

void zynq_device::sysfs_put(const std::string& entry, std::string& err_msg,
    const std::vector<char>& buf)
{
    std::fstream fs = sysfs_open(get_sysfs_path(entry), err_msg, true, true);
    if (!err_msg.empty())
        return;
    fs.write(buf.data(), buf.size());
}

void zynq_device::sysfs_get(const std::string& entry, std::string& err_msg,
    std::vector<char>& buf)
{
    dev_handle dev(sysfs_root);
    dev.open(entry, err_msg, false, true);
    if (!err_msg.empty())
        return;
    dev.get(buf);
}

void zynq_device::sysfs_get(const std::string& entry, std::string& err_msg,
    std::vector<std::string>& sv)
{
    dev_handle dev(sysfs_root);
    dev.open(entry, err_msg, false, true);
    if (!err_msg.empty())
        return;
    dev.get(sv);
}

void zynq_device::sysfs_get(const std::string& entry, std::string& err_msg,
    std::vector<uint64_t>& iv)
{
    uint64_t n;
    std::vector<std::string> sv;

    iv.clear();

    sysfs_get(entry, err_msg, sv);
    if (!err_msg.empty())
        return;

    char *end;
    for (auto& s : sv) {
        std::stringstream ss;

        if (s.empty()) {
            ss << "Reading " << get_sysfs_path(entry) << ", ";
            ss << "can't convert empty string to integer" << std::endl;
            err_msg = ss.str();
            break;
        }
        n = std::strtoull(s.c_str(), &end, 0);
        if (*end != '\0') {
            ss << "Reading " << get_sysfs_path(entry) << ", ";
            ss << "failed to convert string to integer: " << s << std::endl;
            err_msg = ss.str();
            break;
        }
        iv.push_back(n);
    }
}

void zynq_device::sysfs_get(const std::string& entry, std::string& err_msg,
    std::string& s)
{
    std::vector<std::string> sv;

    sysfs_get(entry, err_msg, sv);
    if (!sv.empty())
        s = sv[0];
    else
        s = ""; // default value
}

zynq_device *zynq_device::get_dev()
{
#if __linux__
    // This is based on the fact that on edge devices, we only have one DRM
    // device, which is named as renderD* (eg: renderD128).
    // This path is reliable. It is the same for ARM32 and ARM64.
    static std::string root = "/sys/class/drm/" + get_render_devname() + "/device/";
#else
    static std::string root = "/dev/dri/" + get_render_devname();
#endif
    static zynq_device dev(root);
    return &dev;
}

zynq_device::zynq_device(const std::string& root) : sysfs_root(root)
{
}

std::string
get_render_devname()
{
    static const std::string render_dir{"/dev/dri/"};
    static const std::string render_dev_sym_dir = render_dir + "by-path/";
    std::string render_devname;

    // On Linux Edge platforms 'zyxclmm_drm' is the name of zocl node
    // in device tree A symlink to render device is created based on
    // this node name
    static const std::regex filter{"platform.*zyxclmm_drm-render"};

    try {
        boost::filesystem::directory_iterator end_itr;
        for( boost::filesystem::directory_iterator itr( render_dev_sym_dir ); itr != end_itr; ++itr) {
            if (!std::regex_match(itr->path().filename().string(), filter))
                continue;
            if (boost::filesystem::is_symlink(itr->path()))
                render_devname = boost::filesystem::read_symlink(itr->path()).filename().string();
            break;
        }
    } catch (std::exception& e) {
        /* consume any boost related exceptions, fall through and return the default */
        render_devname.clear();
    }

    if (render_devname.empty())
        render_devname = "renderD128";

    return render_devname;
}
