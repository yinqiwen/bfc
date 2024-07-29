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
#include <cstdint>
#include <functional>
#include "absl/types/span.h"
#include "bfc/cache/address.h"
#include "bfc/common/time_helper.h"
#include "folly/experimental/ReadMostlySharedPtr.h"
#include "folly/synchronization/PicoSpinLock.h"

namespace bfc {

template <typename Ret, typename T>
using VisitFunc = std::function<Ret(T&)>;

template <typename T>
class MemCacheKey {
 public:
  MemCacheKey(const T& key) : key_(key) { control_ = 0; }

  MemCacheKey(const MemCacheKey& other) {
    control_ = other.control_;
    key_ = other.key_;
  }
  ~MemCacheKey() { Clear(); }
  void SetKey(const T& key) {
    key_ = key;
    refreshing_ = 0;
  }
  const T& GetKey() const { return key_; }
  inline void Clear() {
    refreshing_ = 0;
    access_clock_ = 0;
    create_clock_ = 0;
    empty_ = 0;
  }
  bool IsEmpty() const { return empty_; }
  void SetEmpty(bool v) {
    if (v) {
      empty_ = 1;
      key_ = {};
    } else {
      empty_ = 0;
    }
  }
  inline void SetRefreshing(bool v) { refreshing_ = v ? 1 : 0; }
  inline bool IsRefreshing() const { return refreshing_; }
  inline uint32_t GetAccessClock() const { return access_clock_; }
  inline void SetAccessClock(int64_t ts) { access_clock_ = static_cast<uint32_t>(ts) & kClockMax; }
  inline uint32_t GetCreateClock() const { return create_clock_; }
  inline void SetCreateClock(int64_t ts) {
    create_clock_ = static_cast<uint32_t>(ts) & kClockMax;
    // force update refresh state
    refreshing_ = 0;
  }
  inline uint32_t GetAccessIdleTimeSecs() const {
    auto now = GetCurrentClock();
    if (now >= access_clock_) {
      return now - access_clock_;
    } else {
      return now + (kClockMax - access_clock_);
    }
  }
  inline uint32_t GetCreateIdleTimeSecs() const {
    auto now = GetCurrentClock();
    if (now >= create_clock_) {
      return now - create_clock_;
    } else {
      return now + (kClockMax - create_clock_);
    }
  }
  void Update(int64_t ts) {
    SetAccessClock(ts);
    SetCreateClock(ts);
  }

 private:
  static constexpr uint32_t kClockTimeBits = 30;
  static constexpr uint32_t kClockMax = ((1 << kClockTimeBits) - 1);
  using Lock = folly::PicoSpinLock<uint64_t>;

  uint32_t GetCurrentClock() const {
    int64_t ts = gettimeofday_s();
    return static_cast<uint32_t>(ts) & kClockMax;
  }

  union {
    struct {
      uint64_t access_clock_ : kClockTimeBits;
      uint64_t create_clock_ : kClockTimeBits;
      uint64_t refreshing_ : 1;
      uint64_t empty_ : 1;
      uint64_t reserved_ : 2;
      // uint64_t lock_bit_ : 1;
    };
    uint64_t control_;
    // Lock lock_;
  };
  T key_;
};

#pragma pack(4)
class DiskEntryHeader {
 public:
  DiskEntryHeader() {
    control0_ = 0;
    control1_ = 0;
    control2_ = 0;
  }
  void SetPrev(Address addr) { prev_address_ = addr.Control(); }
  void ClearPrev() { prev_address_ = 0; }
  Address GetPrev() const { return Address(prev_address_); }
  uint64_t GetKeySize() const { return key_length_; }
  uint64_t GetValueSize() const { return value_length_; }
  uint64_t GetValueCapacity() const { return value_capacity_; }
  void SetKeySize(uint64_t v) { key_length_ = v; }
  void SetValueSize(uint64_t v) {
    value_length_ = v;
    if (value_capacity_ == 0) {
      value_capacity_ = v;
    }
  }
  inline bool Erased() const { return static_cast<bool>(erased_); }
  inline void SetErased(bool desired) { erased_ = desired; }
  void SetHash(uint64_t hashcode) {
    hash0_ = (hashcode & kMaxHash0);
    hash1_ = ((hashcode >> (64 - kHash1Bits)) & kMaxHash1);
  }
  bool EqualHash(uint64_t hashcode) {
    uint64_t expect_hash0 = (hashcode & kMaxHash0);
    uint64_t expect_hash1 = ((hashcode >> (64 - kHash1Bits)) & kMaxHash1);
    return hash0_ == expect_hash0 && hash1_ == expect_hash1;
  }

 private:
  static constexpr uint32_t kHash0Bits = 64 - Address::kAddressBits - 1;
  static constexpr uint64_t kMaxHash0 = ((1ULL << kHash0Bits) - 1);
  static constexpr uint32_t kHash1Bits = 64 - 2 * Address::kOffsetBits;
  static constexpr uint64_t kMaxHash1 = ((1ULL << kHash1Bits) - 1);
  union {
    struct {
      uint64_t prev_address_ : Address::kAddressBits;
      uint64_t erased_ : 1;
      uint64_t hash0_ : kHash0Bits;
    };
    uint64_t control0_;
  };
  union {
    struct {
      uint64_t key_length_ : Address::kOffsetBits;
      uint64_t value_length_ : Address::kOffsetBits;
      uint64_t hash1_ : kHash1Bits;
    };
    uint64_t control1_;
  };
  union {
    struct {
      uint32_t value_capacity_ : Address::kOffsetBits;
      uint32_t unused_ : 32 - Address::kOffsetBits;
    };
    uint32_t control2_;
  };
};
static_assert(sizeof(DiskEntryHeader) == 20, "sizeof(DiskEntryHeader) != 20");
#pragma pack()

template <typename KeyT, typename ValueT>
struct DiskEntryValue {
  Address addr;
  DiskEntryHeader header;
  KeyT key;
  ValueT value;
  uint64_t create_unix_secs = 0;
};

template <typename T>
struct DiskObjectHelper {
  bool Serialize(const T& obj, std::vector<uint8_t>& buffer) {
    if constexpr (std::is_same_v<T, int64_t> || std::is_same_v<T, uint64_t> || std::is_same_v<T, int32_t> ||
                  std::is_same_v<T, uint32_t> || std::is_same_v<T, int16_t> || std::is_same_v<T, uint16_t> ||
                  std::is_same_v<T, int8_t> || std::is_same_v<T, uint8_t>) {
      const uint8_t* data = reinterpret_cast<const uint8_t*>(&obj);
      buffer.insert(buffer.end(), data, data + sizeof(T));
      return true;
    } else {
      static_assert(sizeof(T) == std::size_t(-1), "");
    }
  }
  bool Parse(T& obj, absl::Span<const uint8_t> buffer) {
    if constexpr (std::is_same_v<T, int64_t> || std::is_same_v<T, uint64_t> || std::is_same_v<T, int32_t> ||
                  std::is_same_v<T, uint32_t> || std::is_same_v<T, int16_t> || std::is_same_v<T, uint16_t> ||
                  std::is_same_v<T, int8_t> || std::is_same_v<T, uint8_t>) {
      const char* data = reinterpret_cast<const char*>(buffer.data());
      memcpy(&obj, data, sizeof(T));
      return true;
    } else {
      static_assert(sizeof(T) == std::size_t(-1), "");
    }
  }
};

template <>
struct DiskObjectHelper<std::string_view> {
  bool Serialize(const std::string_view& obj, std::vector<uint8_t>& buffer) {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(obj.data());
    buffer.insert(buffer.end(), data, data + obj.size());
    return true;
  }
  bool Parse(std::string_view& obj, absl::Span<const uint8_t> buffer) {
    const char* data = reinterpret_cast<const char*>(buffer.data());
    obj = std::string_view(data, buffer.size());
    return true;
  }
};

}  // namespace bfc