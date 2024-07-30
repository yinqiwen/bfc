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
#include "bfc/common/segment_store.h"
namespace bfc {

absl::StatusOr<typename SegmentStore::SmartPtr> SegmentStore::New(const Options& opts) {
  SmartPtr p(new SegmentStore);
  auto status = p->Init(opts);
  if (!status.ok()) {
    return status;
  }
  return p;
}
std::pair<int32_t, int32_t> SegmentStore::GetSegmentIdRange() {
  std::lock_guard<std::mutex> guard(segment_mutex_);
  if (segments_.empty()) {
    return {-1, -1};
  }
  return {static_cast<int32_t>(segments_[0].id), static_cast<int32_t>(segments_[segments_.size() - 1].id)};
}

absl::Status SegmentStore::Init(const Options& opts) {
  opts_ = opts;
  if (opts_.dir.empty()) {
    return absl::InvalidArgumentError("empty 'dir' option");
  }
  if (opts_.start_segment >= 0 && opts_.end_segment >= 0) {
    int32_t end_segment = opts_.end_segment;
    if (opts_.start_segment > end_segment) {
      end_segment = kMaxSegmentId + 1 + end_segment;
    }
    for (int32_t segment = opts_.start_segment; segment <= end_segment; segment++) {
      uint32_t actual_segment_id = static_cast<uint32_t>(segment);
      if (actual_segment_id > kMaxSegmentId) {
        actual_segment_id = actual_segment_id - kMaxSegmentId - 1;
      }
      std::string segment_file_path = GetSegmentFilePath(actual_segment_id);
      auto file_result =
          MmapFile::Open(segment_file_path, nullptr, kMaxSegmentFileSize, PROT_WRITE | PROT_READ, MAP_SHARED, true);
      if (!file_result.ok()) {
        return file_result.status();
      }
      Segment segment_file;
      segment_file.file = std::move(file_result.value());
      segment_file.addr = segment_file.file->template As<uint8_t>();
      segment_file.id = actual_segment_id;
      segment_file.header = reinterpret_cast<SegmentHeader*>(segment_file.addr);
      segment_file.cached_create_unix_secs = segment_file.header->create_unix_secs;
      segments_.emplace_back(std::move(segment_file));
    }
  } else {
    BFC_INFO("Init empty segment store.");
  }
  return absl::OkStatus();
}
std::string SegmentStore::GetSegmentFilePath(uint32_t id) {
  return absl::StrFormat("%s/data.%010d", opts_.dir.c_str(), id);
}

int32_t SegmentStore::GetSegmentIdx(uint32_t id) const {
  if (segments_.empty()) {
    return -1;
  }
  if (segments_[0].id <= segments_[segments_.size() - 1].id) {
    if (id < segments_[0].id) {
      return -1;
    }
    if (id > segments_[segments_.size() - 1].id) {
      return -1;
    }
    return id - segments_[0].id;
  } else {
    if (id >= segments_[0].id) {
      uint32_t idx = id - segments_[0].id;
      return idx < segments_.size() ? static_cast<int32_t>(idx) : -1;
    }
    if (id < segments_[0].id && id <= segments_[segments_.size() - 1].id) {
      uint32_t ridx = segments_[segments_.size() - 1].id - id;
      return segments_.size() - 1 - ridx;
    }
    return -1;
  }
}
bool SegmentStore::IsValidSegment(uint32_t segment) const { return GetSegmentIdx(segment) != -1; }
typename SegmentStore::SegmentHeader* SegmentStore::GetSegment(uint32_t id) {
  std::lock_guard<std::mutex> guard(segment_mutex_);
  int32_t idx = GetSegmentIdx(id);
  if (idx < 0) {
    return nullptr;
  }
  Segment& segment = segments_[idx];
  return segment.header;
}
absl::Status SegmentStore::NewSegment(uint32_t id) {
  std::string segment_file_path = GetSegmentFilePath(id);
  auto file_result =
      MmapFile::Open(segment_file_path, nullptr, kMaxSegmentFileSize, PROT_WRITE | PROT_READ, MAP_SHARED, true);
  if (!file_result.ok()) {
    return file_result.status();
  }
  Segment segment;
  segment.file = std::move(file_result.value());
  segment.addr = segment.file->template As<uint8_t>();
  segment.id = id;
  segment.header = reinterpret_cast<SegmentHeader*>(segment.addr);
  segment.header->create_unix_secs = static_cast<uint64_t>(gettimeofday_s());
  segment.header->write_offset = kSegmentHeaderSize;
  segment.cached_create_unix_secs = segment.header->create_unix_secs;
  BFC_INFO("Segment:{} created.", segment_file_path);

  segments_.emplace_back(std::move(segment));
  if (opts_.max_segments > 0 && segments_.size() > opts_.max_segments) {
    Segment first = std::move(segments_.front());
    segments_.pop_front();
    RemoveSegment(first);
  }
  return absl::OkStatus();
}
uint32_t SegmentStore::RemoveTTLExpiredSegments() {
  if (opts_.segment_ttl_secs <= 0) {
    return 0;
  }
  uint32_t n = 0;
  std::lock_guard<std::mutex> guard(segment_mutex_);
  while (!segments_.empty()) {
    if (segments_.size() == 1) {
      // keep last segment
      return n;
    }
    uint64_t create_unix_secs = segments_[0].header->create_unix_secs;
    uint64_t now_secs = static_cast<uint64_t>(gettimeofday_s());
    if (now_secs <= create_unix_secs) {
      return n;
    }
    if (now_secs - create_unix_secs < opts_.segment_ttl_secs) {
      return n;
    }
    Segment first = std::move(segments_.front());
    segments_.pop_front();
    RemoveSegment(first);
    n++;
  }
  return 0;
}
void SegmentStore::RemoveSegment(Segment& segment) {
  std::string to_delete_file_path = GetSegmentFilePath(segment.id);
  // delay delete
  MmapFile* delete_file = segment.file.release();
  Timer::GetInstance()->Schedule(
      [to_delete_file_path, delete_file]() mutable {
        // remove file
        delete delete_file;
        int rc = ::remove(to_delete_file_path.c_str());
        BFC_INFO("Segment:{} dealy remove with rc:{}", to_delete_file_path, rc);
      },
      std::chrono::seconds(opts_.segment_delete_delay_secs));
}
absl::StatusOr<typename SegmentStore::LogicAddress> SegmentStore::Write(const void* data, size_t len) {
  if (len > kMaxValueSize) {
    return absl::InvalidArgumentError("too large content siz to write");
  }
  std::lock_guard<std::mutex> guard(segment_mutex_);
  bool new_segment = false;
  uint64_t to_write_n = len;
  if (segments_.empty()) {
    new_segment = true;
  } else {
    if (to_write_n + segments_[segments_.size() - 1].header->write_offset > kMaxSegmentFileSize) {
      new_segment = true;
    }
  }
  if (new_segment) {
    uint32_t new_semgnet_id = segments_.empty() ? 0 : segments_[segments_.size() - 1].id + 1;
    if (new_semgnet_id > kMaxSegmentId) {
      new_semgnet_id = 0;
    }
    auto status = NewSegment(new_semgnet_id);
    if (!status.ok()) {
      return status;
    }
  }
  ++updates_;
  auto& write_segment = segments_[segments_.size() - 1];
  LogicAddress addr{write_segment.id, write_segment.header->write_offset};
  memcpy(write_segment.addr + write_segment.header->write_offset, data, len);
  write_segment.header->write_offset += len;
  return addr;
}
absl::Status SegmentStore::Update(LogicAddress addr, const void* data, size_t len) {
  if (len > kMaxValueSize) {
    return absl::InvalidArgumentError("too large content siz to write");
  }
  std::lock_guard<std::mutex> guard(segment_mutex_);
  int32_t idx = GetSegmentIdx(addr.first);
  if (idx < 0) {
    return absl::NotFoundError("no segment found");
  }
  Segment& segment = segments_[idx];
  memcpy(segment.addr + addr.second, data, len);
  ++updates_;
  return absl::OkStatus();
}

absl::StatusOr<uint8_t*> SegmentStore::Get(LogicAddress addr, uint64_t* create_unix_secs) {
  std::lock_guard<std::mutex> guard(segment_mutex_);
  int32_t idx = GetSegmentIdx(addr.first);
  if (idx < 0) {
    return absl::OutOfRangeError("no segment found");
  }
  Segment& segment = segments_[idx];
  if (nullptr != create_unix_secs) {
    *create_unix_secs = segment.cached_create_unix_secs;
  }
  return segment.addr + addr.second;
}

void SegmentStore::Visit(VisitFunc&& f) {
  for (Segment& segment : segments_) {
    f(segment.id, segment.addr, segment.header->write_offset);
  }
}

}  // namespace bfc