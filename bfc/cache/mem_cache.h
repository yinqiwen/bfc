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
#include <memory>
#include <utility>
#include "absl/cleanup/cleanup.h"
#include "absl/status/statusor.h"
#include "folly/FBVector.h"
#include "folly/Random.h"
#include "folly/ThreadCachedInt.h"
#include "folly/ThreadLocal.h"
#include "folly/synchronization/MicroSpinLock.h"
#include "folly/synchronization/PicoSpinLock.h"

#include "bfc/cache/options.h"
#include "bfc/cache/stats.h"
#include "bfc/cache/types.h"
#include "bfc/common/time_helper.h"
#include "bfc/timer/timer.h"

namespace bfc {
template <class KeyT, class ValueT, class HashFcn = std::hash<KeyT>, class EqualFcn = std::equal_to<KeyT>>
class MemCache {
 public:
  using value_type = ValueT;
  using key_type = KeyT;
  using SmartPtr = std::unique_ptr<MemCache>;
  static absl::StatusOr<SmartPtr> New(const CacheOptions& opts);
  absl::Status Put(const KeyT& key, ValueT&& val, uint32_t create_unix_secs = 0);
  absl::StatusOr<ValueT> Get(const KeyT& key, ReadOptions opt = {}, bool* ttl_expired = nullptr);
  size_t Delete(const KeyT& key);
  size_t Size() const { return size_.load(); }
  template <typename Ret>
  absl::StatusOr<Ret> Visit(const KeyT& key, VisitFunc<Ret, value_type>&& f, ReadOptions opt = {},
                            bool* ttl_expired = nullptr);

  Stats& GetStats() { return stats_; }
  ~MemCache();

 private:
  static constexpr uint32_t kClockTimeBits = 30;
  static constexpr uint32_t kArraySizeHint = 8;
  static constexpr uint32_t kLRUSamplePoolSize = 128;
  static constexpr uint32_t kMaxLRUEvictCountPerLoop = 100;
  using KeyLock = folly::PicoSpinLock<uint64_t>;
  using BucketLock = folly::MicroSpinLock;
  struct SampleItem {
    int32_t bucket_idx = -1;
    int32_t list_idx = -1;
    uint32_t idle_unix_secs = 0;
    void Clear() {
      bucket_idx = -1;
      list_idx = -1;
      idle_unix_secs = 0;
    }
  };

  struct Bucket {
    folly::fbvector<MemCacheKey<key_type>> keys;
    folly::fbvector<value_type> values;
  };

  MemCache() = default;
  absl::Status Init(const CacheOptions& opts);

  inline uint32_t GetBucketIndex(uint64_t hashcode) const {
    return static_cast<uint32_t>(hashcode % (buckets_.size()));
  }

  void SampleRoutine();
  void EvictionPoolPopulate();
  void PerformEvictions();
  std::vector<SampleItem> CacheRandomGetN(uint32_t n);
  void Delete(uint32_t bucket_idx, uint32_t list_idx);

  CacheOptions opts_;
  std::vector<Bucket> buckets_;
  std::vector<BucketLock> bucket_locks_;
  HashFcn hash_;
  EqualFcn equal_;
  std::atomic<uint64_t> size_{0};
  value_type empty_value_;

  folly::ThreadLocalPRNG rng_;
  TimerTaskId sample_task_id_;
  bool sampling_ = false;
  folly::ThreadLocal<std::array<SampleItem, kLRUSamplePoolSize>> lru_sample_pool_;

  Stats stats_;
};

template <class K, class V, class H, class E>
absl::StatusOr<typename MemCache<K, V, H, E>::SmartPtr> MemCache<K, V, H, E>::New(const CacheOptions& opts) {
  SmartPtr p(new MemCache);
  auto status = p->Init(opts);
  if (!status.ok()) {
    return status;
  }
  return p;
}

template <class K, class V, class H, class E>
MemCache<K, V, H, E>::~MemCache() {
  sample_task_id_.Cancel();
}
template <class K, class V, class H, class E>
absl::Status MemCache<K, V, H, E>::Init(const CacheOptions& opts) {
  opts_ = opts;
  buckets_.resize(opts_.max_size);
  bucket_locks_.resize(opts_.max_size);
  for (auto& lock : bucket_locks_) {
    lock.init();
  }
  sample_task_id_ = Timer::GetInstance()->ScheduleAtFixedRate(
      std::bind(&MemCache::SampleRoutine, this), std::chrono::milliseconds(opts_.sample_routine_interval_ms));
  return absl::OkStatus();
}

template <class K, class V, class H, class E>
std::vector<typename MemCache<K, V, H, E>::SampleItem> MemCache<K, V, H, E>::CacheRandomGetN(uint32_t n) {
  std::vector<SampleItem> results;
  results.reserve(n);
  uint32_t maxsteps = n * 10;
  uint32_t cursor = 0;
  while (results.size() < n && maxsteps--) {
    auto rand = folly::Random::rand32(0, opts_.max_size, rng_);
    Bucket& bucket = buckets_[rand];
    std::lock_guard<BucketLock> guard(bucket_locks_[rand]);
    uint32_t list_idx = 0;
    for (const auto& key : bucket.keys) {
      list_idx++;
      if (key.IsEmpty()) {
        continue;
      }
      SampleItem item{.bucket_idx = rand, .list_idx = list_idx - 1, .idle_unix_secs = key.GetAccessIdleTimeSecs()};
      if (results.size() < n) {
        results.emplace_back(item);
      } else {
        uint32_t r = folly::Random::rand32(0, opts_.max_size, rng_);
        r = r % (results.size() + 1);
        if (r < results.size()) {
          results[r] = item;
        }
      }
      cursor++;
    }
    if (cursor >= n) {
      return results;
    }
  }
  return results;
}

template <class K, class V, class H, class E>
void MemCache<K, V, H, E>::EvictionPoolPopulate() {
  auto& tls_lru_sample_pool = *lru_sample_pool_;
  std::vector<SampleItem> samples = CacheRandomGetN(opts_.sample_count);
  for (auto& item : samples) {
    // K key = iter->first;
    // if (iter->first == index_config_.emptyKey || iter->first == index_config_.erasedKey ||
    //     iter->first == index_config_.lockedKey) {
    //   continue;
    // }
    // uint32_t cache_index = iter.getIndex();
    uint32_t idle_unix_secs = item.idle_unix_secs;

    uint32_t k = 0;
    while (k < kLRUSamplePoolSize && tls_lru_sample_pool[k].bucket_idx != -1 &&
           tls_lru_sample_pool[k].idle_unix_secs < idle_unix_secs) {
      k++;
    }

    if (k == 0 && tls_lru_sample_pool[kLRUSamplePoolSize - 1].bucket_idx != -1) {
      /* Can't insert if the element is < the worst element we have
       * and there are no empty buckets. */
      continue;
    } else if (k < kLRUSamplePoolSize && tls_lru_sample_pool[k].bucket_idx == -1) {
      /* Inserting into empty position. No setup needed before insert. */
    } else {
      /* Inserting in the middle. Now k points to the first element
       * greater than the element to insert.  */
      auto* tls_lru_sample_pool_ptr = &tls_lru_sample_pool[0];
      if (tls_lru_sample_pool[kLRUSamplePoolSize - 1].bucket_idx == -1) {
        /* Free space on the right? Insert at k shifting
         * all the elements from k to end to the right. */

        memmove(tls_lru_sample_pool_ptr + k + 1, tls_lru_sample_pool_ptr + k,
                sizeof(SampleItem) * (kLRUSamplePoolSize - k - 1));
      } else {
        /* No free space on right? Insert at k-1 */
        k--;
        /* Shift all elements on the left of k (included) to the
         * left, so we discard the element with smaller idle time. */
        memmove(tls_lru_sample_pool_ptr, tls_lru_sample_pool_ptr + 1, sizeof(SampleItem) * k);
      }
    }
    tls_lru_sample_pool[k] = item;
  }
}
template <class K, class V, class H, class E>
void MemCache<K, V, H, E>::PerformEvictions() {
  EvictionPoolPopulate();
  auto& tls_lru_sample_pool = *lru_sample_pool_;
  for (int32_t k = static_cast<int32_t>(kLRUSamplePoolSize) - 1; k >= 0; k--) {
    if (tls_lru_sample_pool[k].bucket_idx == -1) {
      continue;
    }
    auto sample = tls_lru_sample_pool[k];
    tls_lru_sample_pool[k].Clear();
    std::lock_guard<BucketLock> guard(bucket_locks_[sample.bucket_idx]);
    if (buckets_[sample.bucket_idx].keys.size() <= sample.list_idx) {
      continue;
    }
    if (buckets_[sample.bucket_idx].keys[sample.list_idx].GetAccessIdleTimeSecs() >= sample.idle_unix_secs) {
      // delete
      Delete(sample.bucket_idx, sample.list_idx);
      // stats_.lru_evit_count.increment(1);
      stats_.evit_count.increment(1);
      break;
    }
  }
}
template <class K, class V, class H, class E>
void MemCache<K, V, H, E>::SampleRoutine() {
  if (sampling_) {
    return;
  }
  sampling_ = true;
  auto done = absl::MakeCleanup([this]() { sampling_ = false; });
  uint32_t evict_count = 0;
  while (Size() > opts_.max_size && evict_count < kMaxLRUEvictCountPerLoop) {
    PerformEvictions();
    evict_count++;
  }
}
template <class K, class V, class H, class E>
void MemCache<K, V, H, E>::Delete(uint32_t bucket_idx, uint32_t list_idx) {
  if (buckets_[bucket_idx].keys.size() <= kArraySizeHint) {
    buckets_[bucket_idx].keys[list_idx].SetEmpty(true);
    buckets_[bucket_idx].values[list_idx] = {};
  } else {
    buckets_[bucket_idx].keys.erase(buckets_[bucket_idx].keys.begin() + list_idx);
    buckets_[bucket_idx].values.erase(buckets_[bucket_idx].values.begin() + list_idx);
  }
  size_--;
}

template <class K, class V, class H, class E>
absl::Status MemCache<K, V, H, E>::Put(const K& key, V&& val, uint32_t create_unix_secs) {
  uint64_t hashcode = hash_(key);
  uint32_t bucket_idx = GetBucketIndex(hashcode);
  MemCacheKey<K> new_key(key);
  auto now_sec = gettimeofday_s();
  new_key.SetAccessClock(now_sec);
  if (create_unix_secs > 0) {
    new_key.SetCreateClock(create_unix_secs);
  } else {
    new_key.SetCreateClock(now_sec);
  }
  std::lock_guard<BucketLock> guard(bucket_locks_[bucket_idx]);
  int64_t empty_holder_idx = -1;
  for (size_t i = 0; i < buckets_[bucket_idx].keys.size(); i++) {
    auto& key_holder = buckets_[bucket_idx].keys[i];
    if (key_holder.IsEmpty()) {
      if (empty_holder_idx < 0) {
        empty_holder_idx = i;
      }
      continue;
    }
    if (equal_(key_holder.GetKey(), key)) {
      // update
      key_holder.SetRefreshing(false);
      buckets_[bucket_idx].values[i] = std::move(val);
      return absl::OkStatus();
    }
  }
  if (empty_holder_idx >= 0) {
    buckets_[bucket_idx].keys[empty_holder_idx].SetKey(key);
    buckets_[bucket_idx].values[empty_holder_idx] = std::move(val);
  } else {
    buckets_[bucket_idx].keys.emplace_back(new_key);
    buckets_[bucket_idx].values.emplace_back(std::move(val));
  }
  size_++;
  return absl::OkStatus();
}
template <class K, class V, class H, class E>
template <typename Ret>
absl::StatusOr<Ret> MemCache<K, V, H, E>::Visit(const K& key, VisitFunc<Ret, value_type>&& f, ReadOptions opt,
                                                bool* ttl_expired) {
  uint64_t hashcode = hash_(key);
  uint32_t bucket_idx = GetBucketIndex(hashcode);

  std::lock_guard<BucketLock> guard(bucket_locks_[bucket_idx]);
  for (size_t i = 0; i < buckets_[bucket_idx].keys.size(); i++) {
    auto& key_holder = buckets_[bucket_idx].keys[i];
    if (key_holder.IsEmpty()) {
      continue;
    }
    if (equal_(key_holder.GetKey(), key)) {
      if (opt.update_access_timestamp) {
        key_holder.SetAccessClock(gettimeofday_s());
      }
      if (nullptr != ttl_expired) {
        uint32_t create_idle_secs = key_holder.GetCreateIdleTimeSecs();
        bool is_empty_value = buckets_[bucket_idx].values[i] == empty_value_;
        bool ttl_refresh =
            is_empty_value ? create_idle_secs > opts_.empty_item_ttl_secs : create_idle_secs > opts_.ttl_secs;
        if (!key_holder.IsRefreshing()) {
          if (ttl_refresh) {
            key_holder.SetRefreshing(true);
          }
          *ttl_expired = ttl_refresh;
        }
      }
      stats_.cache_hit.increment(1);
      return f(buckets_[bucket_idx].values[i]);
    }
  }
  stats_.cache_missing.increment(1);
  return absl::NotFoundError("");
}

template <class K, class V, class H, class E>
absl::StatusOr<V> MemCache<K, V, H, E>::Get(const K& key, ReadOptions opt, bool* ttl_expired) {
  return Visit<V>(
      key, [&](V& val) -> V { return val; }, opt, ttl_expired);
}
template <class K, class V, class H, class E>
size_t MemCache<K, V, H, E>::Delete(const K& key) {
  uint64_t hashcode = hash_(key);
  uint32_t bucket_idx = GetBucketIndex(hashcode);
  std::lock_guard<BucketLock> guard(bucket_locks_[bucket_idx]);
  for (size_t i = 0; i < buckets_[bucket_idx].keys.size(); i++) {
    auto& key_holder = buckets_[bucket_idx].keys[i];
    if (equal_(key_holder.GetKey(), key)) {
      Delete(bucket_idx, i);
      return 1;
    }
  }
  return 0;
}

}  // namespace bfc