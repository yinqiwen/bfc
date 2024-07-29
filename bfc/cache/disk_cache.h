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
#include <algorithm>
#include <atomic>
#include <filesystem>
#include <memory>
#include <system_error>
#include <tuple>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/status/statusor.h"
#include "folly/FBVector.h"
#include "folly/File.h"
#include "folly/FileUtil.h"
#include "folly/Random.h"
#include "folly/ThreadLocal.h"
#include "folly/detail/AtomicHashUtils.h"
#include "folly/synchronization/MicroSpinLock.h"
#include "folly/synchronization/PicoSpinLock.h"

#include "bfc/cache/hash_bucket.h"
#include "bfc/cache/options.h"
#include "bfc/cache/stats.h"
#include "bfc/cache/types.h"
#include "bfc/common/mmap_helper.h"
#include "bfc/common/rate_limit_file.h"
#include "bfc/common/segment_store.h"
#include "bfc/common/time_helper.h"
#include "bfc/log/log.h"
#include "bfc/timer/timer.h"

namespace bfc {
template <class KeyT, class ValueT, class HashFcn = std::hash<KeyT>, class EqualFcn = std::equal_to<KeyT>>
class DiskCache {
 public:
  using value_type = ValueT;
  using key_type = KeyT;
  using SmartPtr = std::unique_ptr<DiskCache>;
  static absl::StatusOr<SmartPtr> New(const CacheOptions& opts);
  static bool IsEmpty(const std::string& dir);
  absl::Status Put(const KeyT& key, ValueT&& val);
  absl::StatusOr<ValueT> Get(const KeyT& key, uint64_t* create_unix_secs = nullptr);
  size_t Delete(const KeyT& key);
  absl::Status Save();

  Stats& GetStats() { return stats_; }

  ~DiskCache();

 private:
  static constexpr uint32_t kCacheHeaderSize = 1024;
  static constexpr uint32_t kDiskEntryHeaderLen = sizeof(DiskEntryHeader);
  static constexpr uint32_t kSegmentHeaderSize = 64;
  static constexpr uint32_t kMaxSegmentFileSize = 64 * 1024 * 1024;
  static constexpr uint32_t kMaxValueSize = (64 * 1024 * 1024 - kDiskEntryHeaderLen - kSegmentHeaderSize);
  struct DiskCacheHeader {
    size_t num_buckets = 0;
    size_t total_bytes = 0;
    int32_t start_segment = -1;
    int32_t end_segment = -1;
  };

  using BucketLock = HashBucketOverflowEntry::Lock;

  DiskCache() = default;
  absl::Status Init(const CacheOptions& opts);
  absl::Status Load();
  inline uint32_t GetBucketIndex(uint64_t hashcode) const {
    return static_cast<uint32_t>(hashcode % (cache_header_->num_buckets));
  }
  absl::Status NewSegment(uint32_t id);
  absl::StatusOr<Address> Write(const std::vector<uint8_t>& content);
  absl::Status UpdateValue(Address addr, DiskEntryHeader exist_header, const std::vector<uint8_t>& content);

  std::string GetIndexFilePath(bool tmp);

  absl::StatusOr<DiskEntryValue<key_type, value_type>> GetEntry(const KeyT& key, uint64_t hashcode, Address start_addr,
                                                                bool with_value);
  void PutEntry(uint64_t hashcode, Address addr, uint8_t* entry_ptr, DiskEntryHeader& header);

  void SampleRoutine();

  CacheOptions opts_;
  uint8_t* cache_index_buffer_ = nullptr;
  DiskCacheHeader* cache_header_ = nullptr;
  HashBucket* buckets_ = nullptr;

  HashFcn hash_;
  EqualFcn equal_;

  typename SegmentStore::SmartPtr segment_store_;
  uint64_t last_save_updates_ = 0;

  folly::ThreadLocalPRNG rng_;
  TimerTaskId sample_task_id_;
  bool sampling_ = false;

  Stats stats_;
};
template <class K, class V, class H, class E>
bool DiskCache<K, V, H, E>::IsEmpty(const std::string& dir) {
  std::string path = absl::StrFormat("%s/index", dir.c_str());
  std::error_code err;
  bool r = std::filesystem::exists(path, err);
  return r;
}

template <class K, class V, class H, class E>
absl::StatusOr<typename DiskCache<K, V, H, E>::SmartPtr> DiskCache<K, V, H, E>::New(const CacheOptions& opts) {
  SmartPtr p(new DiskCache);
  auto status = p->Init(opts);
  if (!status.ok()) {
    return status;
  }
  return p;
}

template <class K, class V, class H, class E>
DiskCache<K, V, H, E>::~DiskCache() {
  sample_task_id_.Cancel();
  delete[] cache_index_buffer_;
}
template <class K, class V, class H, class E>
absl::Status DiskCache<K, V, H, E>::Init(const CacheOptions& opts) {
  opts_ = opts;
  if (opts_.dir.empty()) {
    return absl::InvalidArgumentError("empty 'dir' option");
  }
  auto status = Load();
  if (!status.ok()) {
    delete[] cache_index_buffer_;
    size_t num_buckets = size_t(opts_.max_size / HashBucket::kNumEntries);
    size_t total_bytes = sizeof(HashBucket) * num_buckets + kCacheHeaderSize;
    cache_index_buffer_ = new uint8_t[total_bytes];
    memset(cache_index_buffer_, 0, total_bytes);
    cache_header_ = reinterpret_cast<DiskCacheHeader*>(cache_index_buffer_);
    cache_header_->num_buckets = num_buckets;
    cache_header_->total_bytes = total_bytes;
    cache_header_->start_segment = -1;
    cache_header_->end_segment = -1;
    buckets_ = reinterpret_cast<HashBucket*>(cache_index_buffer_ + kCacheHeaderSize);
    SegmentStore::Options segment_store_opts;
    segment_store_opts.dir = opts_.dir;
    segment_store_opts.max_segments = opts_.max_segments;
    segment_store_opts.segment_delete_delay_secs = opts_.segment_delete_delay_secs;
    segment_store_opts.segment_ttl_secs = opts_.segment_ttl_secs;
    auto segment_store_result = SegmentStore::New(segment_store_opts);
    if (!segment_store_result.ok()) {
      return segment_store_result.status();
    }
    segment_store_ = std::move(segment_store_result.value());
  }

  sample_task_id_ = Timer::GetInstance()->ScheduleAtFixedRate(
      std::bind(&DiskCache::SampleRoutine, this), std::chrono::milliseconds(opts_.sample_routine_interval_ms));
  return absl::OkStatus();
}

template <class K, class V, class H, class E>
absl::Status DiskCache<K, V, H, E>::Load() {
  std::string index_fpath = GetIndexFilePath(false);
  if (!std::filesystem::exists(std::filesystem::path(index_fpath))) {
    BFC_WARN("Index file:{} is not exist", index_fpath);
    return absl::NotFoundError("");
  }
  try {
    int file_flags = O_RDONLY | O_CLOEXEC;
    int mode = 0644;
    auto index_file = std::make_unique<folly::File>(index_fpath, file_flags, mode);
    struct stat st;
    int rc = fstat(index_file->fd(), &st);
    if (rc != 0) {
      int err = errno;
      return absl::ErrnoToStatus(err, "fstat failed for file path:" + index_fpath);
    }
    cache_index_buffer_ = new uint8_t[st.st_size];
    ssize_t n = folly::readFull(index_file->fd(), cache_index_buffer_, st.st_size);
    if (n != st.st_size) {
      int err = errno;
      return absl::ErrnoToStatus(err, "read index");
    }
    cache_header_ = reinterpret_cast<DiskCacheHeader*>(cache_index_buffer_);
    buckets_ = reinterpret_cast<HashBucket*>(cache_index_buffer_ + kCacheHeaderSize);
    SegmentStore::Options segment_store_opts;
    segment_store_opts.dir = opts_.dir;
    segment_store_opts.max_segments = opts_.max_segments;
    segment_store_opts.segment_delete_delay_secs = opts_.segment_delete_delay_secs;
    segment_store_opts.segment_ttl_secs = opts_.segment_ttl_secs;
    segment_store_opts.end_segment = cache_header_->end_segment;
    segment_store_opts.start_segment = cache_header_->start_segment;
    auto segment_store_result = SegmentStore::New(segment_store_opts);
    if (!segment_store_result.ok()) {
      return segment_store_result.status();
    }
    segment_store_ = std::move(segment_store_result.value());

    size_t num_buckets = size_t(opts_.max_size / HashBucket::kNumEntries);
    if (cache_header_->num_buckets != num_buckets) {
      BFC_WARN("Rehash the whole disk cache index since num_buckets changed from {} to {}", cache_header_->num_buckets,
               num_buckets);
      delete[] cache_index_buffer_;
      size_t num_buckets = size_t(opts_.max_size / HashBucket::kNumEntries);
      size_t total_bytes = sizeof(HashBucket) * num_buckets + kCacheHeaderSize;
      cache_index_buffer_ = new uint8_t[total_bytes];
      memset(cache_index_buffer_, 0, total_bytes);
      cache_header_ = reinterpret_cast<DiskCacheHeader*>(cache_index_buffer_);
      cache_header_->num_buckets = num_buckets;
      cache_header_->total_bytes = total_bytes;
      auto segment_id_range = segment_store_->GetSegmentIdRange();
      cache_header_->start_segment = segment_id_range.first;
      cache_header_->end_segment = segment_id_range.second;
      buckets_ = reinterpret_cast<HashBucket*>(cache_index_buffer_ + kCacheHeaderSize);

      segment_store_->Visit([&](uint32_t id, uint8_t* ptr, uint32_t len) {
        uint32_t offset = SegmentStore::kSegmentHeaderSize;
        while (offset < len) {
          DiskEntryHeader header;
          memcpy(&header, ptr + offset, sizeof(header));
          if (header.Erased()) {
            offset += (sizeof(DiskEntryHeader) + header.GetKeySize() + header.GetValueCapacity());
            continue;
          }
          DiskObjectHelper<K> key_helper;
          absl::Span<const uint8_t> key_view(ptr + offset + kDiskEntryHeaderLen, header.GetKeySize());
          K key;
          if (!key_helper.Parse(key, key_view)) {
            BFC_ERROR("invalid segment:{} to parse key at offset:{}", id, offset);
            return;
          }
          if (header.GetPrev().Control() != 0) {
            header.ClearPrev();
            memcpy(ptr + offset, &header, sizeof(header));
          }
          uint64_t hashcode = hash_(key);
          Address addr(id, offset);
          PutEntry(hashcode, addr, ptr + offset, header);
          offset += (sizeof(DiskEntryHeader) + header.GetKeySize() + header.GetValueCapacity());
        }
      });
    }
  } catch (...) {
    return absl::InvalidArgumentError("Load index or segments.");
  }
  return absl::OkStatus();
}
template <class K, class V, class H, class E>
absl::Status DiskCache<K, V, H, E>::Save() {
  uint64_t current_updates = segment_store_->GetUpdates();
  if (current_updates == 0 || last_save_updates_ == current_updates) {
    return absl::AlreadyExistsError("no updates since last save");
  }
  std::string tmp_index_fpath = GetIndexFilePath(true);
  try {
    int file_flags = O_RDWR | O_CREAT | O_CLOEXEC | O_TRUNC;
    int mode = 0644;
    auto file = std::make_unique<folly::File>(tmp_index_fpath, file_flags, mode);
    auto segment_id_range = segment_store_->GetSegmentIdRange();
    cache_header_->start_segment = segment_id_range.first;
    cache_header_->end_segment = segment_id_range.second;
    RateLimitFile rate_limit_saving(file->fd(), opts_.save_limit_mb_bytes_per_10ms * 1024 * 1024);
    ssize_t n = rate_limit_saving.WriteFull(cache_index_buffer_, cache_header_->total_bytes);
    if (n != static_cast<ssize_t>(cache_header_->total_bytes)) {
      int err = errno;
      return absl::ErrnoToStatus(err, "write index content");
    }

    file->close();
    std::string index_fpath = GetIndexFilePath(false);
    int rc = ::rename(tmp_index_fpath.c_str(), index_fpath.c_str());
    if (0 != rc) {
      int err = errno;
      ::remove(tmp_index_fpath.c_str());
      return absl::ErrnoToStatus(err, "save index file");
    }
    last_save_updates_ = current_updates;
    return absl::OkStatus();
  } catch (...) {
    int err = errno;
    return absl::ErrnoToStatus(err, "create index file");
  }
}

template <class K, class V, class H, class E>
void DiskCache<K, V, H, E>::SampleRoutine() {
  if (sampling_) {
    return;
  }
  sampling_ = true;
  auto done = absl::MakeCleanup([this]() { sampling_ = false; });
  uint32_t n = segment_store_->RemoveTTLExpiredSegments();
  stats_.evit_count.increment(n);
}

template <class K, class V, class H, class E>
std::string DiskCache<K, V, H, E>::GetIndexFilePath(bool tmp) {
  if (tmp) {
    return absl::StrFormat("%s/index.tmp", opts_.dir.c_str());
  } else {
    return absl::StrFormat("%s/index", opts_.dir.c_str());
  }
}
template <class K, class V, class H, class E>
void DiskCache<K, V, H, E>::PutEntry(uint64_t hashcode, Address addr, uint8_t* entry_ptr, DiskEntryHeader& header) {
  uint32_t bucket_idx = GetBucketIndex(hashcode);
  HashBucketEntry* unused_entry = nullptr;
  for (auto& entry : buckets_[bucket_idx].entries) {
    if (entry.Unused()) {
      if (unused_entry == nullptr) {
        unused_entry = &entry;
      }
    } else {
      if (entry.EqualHash(hashcode)) {
        header.SetPrev(entry.GetAddress());
        entry.SetAddress(addr);
        memcpy(entry_ptr, &header, sizeof(header));
        return;
      }
    }
  }

  if (unused_entry != nullptr) {
    unused_entry->SetHash(hashcode);
    unused_entry->SetAddress(addr);
    return;
  }
  if (!buckets_[bucket_idx].overflow_entry.Unused()) {
    header.SetPrev(buckets_[bucket_idx].overflow_entry.GetAddress());
    memcpy(entry_ptr, &header, sizeof(header));
  }
  buckets_[bucket_idx].overflow_entry.SetAddress(addr);
}

template <class K, class V, class H, class E>
absl::Status DiskCache<K, V, H, E>::UpdateValue(Address addr, DiskEntryHeader exist_header,
                                                const std::vector<uint8_t>& content) {
  auto ptr_result = segment_store_->Get({addr.Segment(), addr.Offset()});
  if (!ptr_result.ok()) {
    return ptr_result.status();
  }
  if (exist_header.Erased()) {
    memcpy(ptr_result.value(), &exist_header, sizeof(exist_header));
  } else {
    uint32_t new_val_size = content.size() - sizeof(DiskEntryHeader) - exist_header.GetKeySize();
    exist_header.SetValueSize(new_val_size);
    memcpy(ptr_result.value(), &exist_header, sizeof(exist_header));
    memcpy(ptr_result.value() + sizeof(DiskEntryHeader) + exist_header.GetKeySize(),
           content.data() + sizeof(DiskEntryHeader) + exist_header.GetKeySize(), new_val_size);
  }

  return absl::OkStatus();
}

template <class K, class V, class H, class E>
absl::StatusOr<Address> DiskCache<K, V, H, E>::Write(const std::vector<uint8_t>& content) {
  if (content.size() > kMaxValueSize) {
    return absl::InvalidArgumentError("too large content size");
  }
  auto result = segment_store_->Write(content.data(), content.size());
  if (!result.ok()) {
    return result.status();
  }
  Address addr(result.value().first, result.value().second);
  return addr;
}
template <class K, class V, class H, class E>
absl::StatusOr<DiskEntryValue<K, V>> DiskCache<K, V, H, E>::GetEntry(const K& key, uint64_t hashcode,
                                                                     Address start_addr, bool with_value) {
  if (start_addr.Control() == 0) {
    return absl::NotFoundError("");
  }
  uint64_t create_unix_secs;
  auto result = segment_store_->Get({start_addr.Segment(), start_addr.Offset()}, &create_unix_secs);
  if (!result.ok()) {
    return absl::OutOfRangeError("");
  }
  DiskEntryHeader header;
  uint8_t* entry_ptr = result.value();
  memcpy(&header, entry_ptr, sizeof(header));
  if (header.GetKeySize() > kMaxValueSize || header.GetValueSize() > kMaxValueSize) {
    return absl::DataLossError("too large value len to read");
  }

  if (!header.EqualHash(hashcode) || header.Erased()) {
    return GetEntry(key, hashcode, header.GetPrev(), with_value);
  } else {
    absl::Span<const uint8_t> key_view;
    absl::Span<const uint8_t> value_view;
    key_view = absl::Span<const uint8_t>(entry_ptr + kDiskEntryHeaderLen, header.GetKeySize());
    if (with_value) {
      value_view =
          absl::Span<const uint8_t>(entry_ptr + kDiskEntryHeaderLen + header.GetKeySize(), header.GetValueSize());
    }
    DiskObjectHelper<K> key_helper;
    DiskEntryValue<K, V> entry;
    if (!key_helper.Parse(entry.key, key_view)) {
      return absl::DataLossError("parse key failed");
    }
    if (with_value) {
      DiskObjectHelper<V> value_helper;
      if (!value_helper.Parse(entry.value, value_view)) {
        return absl::DataLossError("parse value failed");
      }
    }
    if (equal_(entry.key, key)) {
      entry.addr = start_addr;
      entry.header = header;
      entry.create_unix_secs = create_unix_secs;
      return entry;
    } else {
      return GetEntry(key, hashcode, header.GetPrev(), with_value);
    }
  }
}

template <class K, class V, class H, class E>
absl::Status DiskCache<K, V, H, E>::Put(const K& key, V&& val) {
  uint64_t hashcode = hash_(key);
  uint32_t bucket_idx = GetBucketIndex(hashcode);

  DiskEntryHeader disk_entry;
  std::vector<uint8_t> content;
  content.resize(sizeof(DiskEntryHeader));
  DiskObjectHelper<K> key_serializer;
  if (!key_serializer.Serialize(key, content)) {
    return absl::InvalidArgumentError("serialize key failed");
  }
  disk_entry.SetKeySize(content.size() - sizeof(DiskEntryHeader));
  DiskObjectHelper<V> value_serializer;
  if (!value_serializer.Serialize(val, content)) {
    return absl::InvalidArgumentError("serialize value failed");
  }
  disk_entry.SetValueSize(content.size() - sizeof(DiskEntryHeader) - disk_entry.GetKeySize());
  disk_entry.SetHash(hashcode);

  HashBucketEntry* unused_entry = nullptr;
  std::lock_guard<BucketLock> guard(buckets_[bucket_idx].overflow_entry.GetLock());
  for (auto& entry : buckets_[bucket_idx].entries) {
    if (entry.Unused()) {
      if (unused_entry == nullptr) {
        unused_entry = &entry;
      }
    } else {
      if (entry.EqualHash(hashcode)) {
        if (!opts_.append_only) {
          auto disk_entry_result = GetEntry(key, hashcode, entry.GetAddress(), false);
          if (disk_entry_result.ok()) {
            DiskEntryValue<K, V> exist_disk_entry = std::move(disk_entry_result.value());
            if (exist_disk_entry.header.GetValueSize() >= disk_entry.GetValueSize()) {
              return UpdateValue(exist_disk_entry.addr, exist_disk_entry.header, content);
            }
          }
        }
        disk_entry.SetPrev(entry.GetAddress());
        memcpy(&content[0], &disk_entry, sizeof(disk_entry));
        auto write_result = Write(content);
        if (!write_result.ok()) {
          return write_result.status();
        }
        Address write_addr = write_result.value();
        entry.SetAddress(write_addr);
        return absl::OkStatus();
      }
    }
  }
  if (!opts_.append_only) {
    auto disk_entry_result = GetEntry(key, hashcode, buckets_[bucket_idx].overflow_entry.GetAddress(), false);
    if (disk_entry_result.ok()) {
      DiskEntryValue<K, V> exist_disk_entry = std::move(disk_entry_result.value());
      if (exist_disk_entry.header.GetValueSize() >= disk_entry.GetValueSize()) {
        return UpdateValue(exist_disk_entry.addr, exist_disk_entry.header, content);
      }
    }
  }
  if (unused_entry != nullptr) {
    memcpy(&content[0], &disk_entry, sizeof(disk_entry));
    auto write_result = Write(content);
    if (!write_result.ok()) {
      return write_result.status();
    }
    unused_entry->SetHash(hashcode);
    unused_entry->SetAddress(write_result.value());
    return absl::OkStatus();
  }
  if (!buckets_[bucket_idx].overflow_entry.Unused()) {
    disk_entry.SetPrev(buckets_[bucket_idx].overflow_entry.GetAddress());
  }
  memcpy(&content[0], &disk_entry, sizeof(disk_entry));
  auto write_result = Write(content);
  if (!write_result.ok()) {
    return write_result.status();
  }
  Address write_addr = write_result.value();
  buckets_[bucket_idx].overflow_entry.SetAddress(write_addr);
  return absl::OkStatus();
}

template <class K, class V, class H, class E>
absl::StatusOr<V> DiskCache<K, V, H, E>::Get(const K& key, uint64_t* create_unix_secs) {
  uint64_t hashcode = hash_(key);
  uint32_t bucket_idx = GetBucketIndex(hashcode);
  std::lock_guard<BucketLock> guard(buckets_[bucket_idx].overflow_entry.GetLock());
  for (const auto& entry : buckets_[bucket_idx].entries) {
    if (entry.Unused()) {
      continue;
    } else {
      if (entry.EqualHash(hashcode)) {
        auto disk_entry_result = GetEntry(key, hashcode, entry.GetAddress(), true);
        if (disk_entry_result.ok()) {
          DiskEntryValue<K, V> exist_disk_entry = std::move(disk_entry_result.value());
          if (nullptr != create_unix_secs) {
            *create_unix_secs = exist_disk_entry.create_unix_secs;
          }
          stats_.cache_hit.increment(1);
          return std::move(exist_disk_entry.value);
        }
      }
    }
  }
  auto disk_entry_result = GetEntry(key, hashcode, buckets_[bucket_idx].overflow_entry.GetAddress(), false);
  if (disk_entry_result.ok()) {
    DiskEntryValue<K, V> exist_disk_entry = std::move(disk_entry_result.value());
    if (nullptr != create_unix_secs) {
      *create_unix_secs = exist_disk_entry.create_unix_secs;
    }
    stats_.cache_hit.increment(1);
    return std::move(exist_disk_entry.value);
  }
  stats_.cache_missing.increment(1);
  return absl::NotFoundError("");
}

template <class K, class V, class H, class E>
size_t DiskCache<K, V, H, E>::Delete(const K& key) {
  uint64_t hashcode = hash_(key);
  uint32_t bucket_idx = GetBucketIndex(hashcode);
  std::lock_guard<BucketLock> guard(buckets_[bucket_idx].overflow_entry.GetLock());
  for (const auto& entry : buckets_[bucket_idx].entries) {
    if (entry.Unused()) {
      continue;
    } else {
      if (entry.EqualHash(hashcode)) {
        auto disk_entry_result = GetEntry(key, hashcode, entry.GetAddress(), true);
        if (disk_entry_result.ok()) {
          DiskEntryValue<K, V> exist_disk_entry = std::move(disk_entry_result.value());
          exist_disk_entry.header.SetErased(true);
          auto status = UpdateValue(exist_disk_entry.addr, exist_disk_entry.header, {});
          return status.ok() ? 1 : 0;
        }
      }
    }
  }
  auto disk_entry_result = GetEntry(key, hashcode, buckets_[bucket_idx].overflow_entry.GetAddress(), false);
  if (disk_entry_result.ok()) {
    DiskEntryValue<K, V> exist_disk_entry = std::move(disk_entry_result.value());
    exist_disk_entry.header.SetErased(true);
    auto status = UpdateValue(exist_disk_entry.addr, exist_disk_entry.header, {});
    return status.ok() ? 1 : 0;
  }
  return 0;
}

}  // namespace bfc