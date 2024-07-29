/*
** BSD 3-Clause License
**
** Copyright (c) 2023, qiyingwang <qiyingwang@tencent.com>, the respective contributors, as shown by the AUTHORS file.
** All rights reserved.
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
** * Redistributions of source code must retain the above copyright notice, this
** list of conditions and the following disclaimer.
**
** * Redistributions in binary form must reproduce the above copyright notice,
** this list of conditions and the following disclaimer in the documentation
** and/or other materials provided with the distribution.
**
** * Neither the name of the copyright holder nor the names of its
** contributors may be used to endorse or promote products derived from
** this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "bfc/common/mmap_helper.h"
#include <fmt/core.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "folly/File.h"
namespace bfc {
absl::StatusOr<std::unique_ptr<MmapFile>> MmapFile::Open(const std::string& path, void* fixed_addr, size_t size,
                                                         int prot, int flags, bool own) {
  std::unique_ptr<MmapFile> p(new MmapFile);
  auto status = p->Init(path, fixed_addr, size, prot, flags, own);
  if (!status.ok()) {
    return status;
  }
  return absl::StatusOr<std::unique_ptr<MmapFile>>(std::move(p));
}
MmapFile::MmapFile() {}
MmapFile::~MmapFile() {
  if (nullptr != mapping_addr_ && own_) {
    munmap(mapping_addr_, size_);
  }
}
absl::Status MmapFile::Init(const std::string& path, void* fixed_addr, size_t size, int prot, int flags, bool own) {
  own_ = own;
  std::unique_ptr<folly::File> segment_file;
  try {
    int file_flags = 0;
    if (prot | PROT_WRITE) {
      file_flags = O_RDWR | O_CREAT | O_CLOEXEC;
    } else {
      file_flags = O_RDONLY | O_CLOEXEC;
    }
    int mode = 0644;
    segment_file = std::make_unique<folly::File>(path, file_flags, mode);
    if (size > 0) {
      int rc = ftruncate(segment_file->fd(), size);
      if (rc != 0) {
        int err = errno;
        return absl::ErrnoToStatus(err, "truncate failed for  file path:" + path);
      }
    }
  } catch (...) {
    return absl::InvalidArgumentError("invalid segment file path:" + path);
  }
  struct stat st;
  int rc = fstat(segment_file->fd(), &st);
  if (rc != 0) {
    int err = errno;
    return absl::ErrnoToStatus(err, "fstat failed for  file path:" + path);
  }
  if (st.st_size == 0) {
    return absl::OkStatus();
  }

  if (nullptr != fixed_addr) {
    flags = (flags | MAP_FIXED);
  }
  //   fmt::print("{}, size:{}, st_blocks:{}\n", path, st.st_size, st.st_blocks);
  auto temp_addr = mmap(fixed_addr, st.st_size, prot, flags, segment_file->fd(), 0);
  if (temp_addr == MAP_FAILED) {
    return absl::ErrnoToStatus(errno, "unavailabel address");
  }
  mapping_addr_ = reinterpret_cast<char*>(temp_addr);
  size_ = st.st_size;
  return absl::OkStatus();
}
}  // namespace bfc