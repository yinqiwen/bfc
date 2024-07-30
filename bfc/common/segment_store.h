/*
** BSD 3-Clause License
**
** Copyright (c) 2024, qiyingwang <qiyingwang@tencent.com>, the respective contributors, as shown by the AUTHORS file.
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

#pragma once
#include <deque>
#include <functional>
#include <memory>
#include <mutex>

#include "absl/status/statusor.h"

#include "folly/File.h"
#include "folly/FileUtil.h"

#include "bfc/common/mmap_helper.h"
#include "bfc/common/time_helper.h"
#include "bfc/log/log.h"
#include "bfc/timer/timer.h"

namespace bfc {
class SegmentStore {
 public:
  static constexpr uint32_t kSegmentHeaderSize = 64;
  using LogicAddress = std::pair<uint32_t, uint32_t>;
  using VisitFunc = std::function<void(uint32_t, uint8_t* ptr, uint32_t len)>;
  struct Options {
    std::string dir;
    uint32_t max_segments = 0;  // 0 means no limit
    uint32_t segment_delete_delay_secs = 15;
    uint32_t segment_ttl_secs = 24 * 3600;  // 1day
    int32_t start_segment = -1;
    int32_t end_segment = -1;
  };
  using SmartPtr = std::unique_ptr<SegmentStore>;
  static absl::StatusOr<SmartPtr> New(const Options& opts);
  absl::StatusOr<LogicAddress> Write(const void* data, size_t len);
  absl::Status Update(LogicAddress addr, const void* data, size_t len);
  absl::StatusOr<uint8_t*> Get(LogicAddress addr, uint64_t* create_unix_secs = nullptr);
  std::pair<int32_t, int32_t> GetSegmentIdRange();
  uint32_t RemoveTTLExpiredSegments();
  uint64_t GetUpdates() const { return updates_.load(); }

  void Visit(VisitFunc&& f);

 private:
  static constexpr uint32_t kMaxSegmentFileSize = 64 * 1024 * 1024;
  static constexpr uint32_t kSegmentBits = 22;
  static constexpr uint32_t kMaxSegmentId = ((1ULL << kSegmentBits) - 1);
  // static constexpr uint32_t kMaxSegmentId = 127;
  static constexpr uint32_t kMaxValueSize = kMaxSegmentFileSize - kSegmentHeaderSize;

  struct SegmentHeader {
    uint64_t create_unix_secs = 0;
    uint32_t write_offset = 0;
  };

  struct Segment {
    std::unique_ptr<MmapFile> file;
    uint32_t id = 0;
    uint8_t* addr = nullptr;
    SegmentHeader* header = nullptr;
    uint64_t cached_create_unix_secs = 0;
  };
  SegmentStore() = default;
  absl::Status Init(const Options& opts);

  int32_t GetSegmentIdx(uint32_t id) const;
  bool IsValidSegment(uint32_t segment) const;
  SegmentHeader* GetSegment(uint32_t segment);

  void RemoveSegment(Segment& segment);
  std::string GetSegmentFilePath(uint32_t id);
  absl::Status NewSegment(uint32_t id);

  Options opts_;
  std::atomic<uint64_t> updates_{0};
  std::deque<Segment> segments_;
  std::mutex segment_mutex_;
};
}  // namespace bfc