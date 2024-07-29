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

#include <atomic>
#include <cassert>
#include <cstdint>
#include <thread>

#include "folly/synchronization/PicoSpinLock.h"

#include "bfc/cache/address.h"

namespace bfc {

static_assert(Address::kAddressBits == 48, "Address::kAddressBits != 48");

/// Entry stored in a hash bucket. Packed into 8 bytes.
struct HashBucketEntry {
  /// Invalid value in the hash table
  static constexpr uint64_t kInvalidEntry = 0;
  static constexpr uint32_t kHashBits = 64 - Address::kAddressBits - 1;
  static constexpr uint64_t kMaxHash = ((1ULL << kHashBits) - 1);
  using Lock = folly::PicoSpinLock<uint64_t>;
  HashBucketEntry() : control_{0} {}
  HashBucketEntry(Address address, uint16_t tag) : address_{address.Control()}, hash_{tag}, lock_bit_(0) {}
  HashBucketEntry(uint64_t code) : control_{code} {}
  HashBucketEntry(const HashBucketEntry& other) : control_{other.control_} {}

  inline HashBucketEntry& operator=(const HashBucketEntry& other) {
    control_ = other.control_;
    return *this;
  }
  inline bool operator==(const HashBucketEntry& other) const { return control_ == other.control_; }
  inline bool operator!=(const HashBucketEntry& other) const { return control_ != other.control_; }
  inline bool Unused() const { return address_ == 0; }
  inline Address GetAddress() const { return Address{address_}; }
  inline void SetAddress(Address addr) { address_ = addr.Control(); }
  uint32_t Contorl() const { return control_; }
  void SetHash(uint64_t hashcode) { hash_ = (hashcode & kMaxHash); }
  bool EqualHash(uint64_t hashcode) const {
    uint64_t expect_hash0 = (hashcode & kMaxHash);
    return hash_ == expect_hash0;
  }
  inline Lock& GetLock() { return lock_; }

  union {
    struct {
      uint64_t address_ : 48;  // corresponds to logical address
      uint64_t hash_ : kHashBits;
      uint64_t lock_bit_ : 1;
    };
    uint64_t control_;
    Lock lock_;
  };
};
static_assert(sizeof(HashBucketEntry) == 8, "sizeof(HashBucketEntry) != 8");

/// Entry stored in a hash bucket that points to the next overflow bucket (if any).
struct HashBucketOverflowEntry {
  using Lock = folly::PicoSpinLock<uint64_t>;
  HashBucketOverflowEntry() : control_{0} {}
  HashBucketOverflowEntry(Address address)
      : address_{address.Control()}, unused_{0}, erased_(0), tentative_(0), lock_bit_(0) {}
  HashBucketOverflowEntry(const HashBucketOverflowEntry& other) : control_{other.control_} {}

  inline HashBucketOverflowEntry& operator=(const HashBucketOverflowEntry& other) {
    control_ = other.control_;
    return *this;
  }
  inline bool operator==(const HashBucketOverflowEntry& other) const { return control_ == other.control_; }
  inline bool operator!=(const HashBucketOverflowEntry& other) const { return control_ != other.control_; }
  inline bool Unused() const { return address_ == 0; }
  inline Address GetAddress() const { return Address{address_}; }
  inline void SetAddress(Address addr) { address_ = addr.Control(); }
  inline bool Erased() const { return static_cast<bool>(erased_); }
  inline void SetErased(bool desired) { erased_ = desired; }
  inline bool Tentative() const { return static_cast<bool>(tentative_); }
  inline void SetTentative(bool desired) { tentative_ = desired; }
  inline Lock& GetLock() { return lock_; }

  union {
    struct {
      uint64_t address_ : Address::kAddressBits;
      uint64_t unused_ : 13;
      uint64_t erased_ : 1;
      uint64_t tentative_ : 1;
      uint64_t lock_bit_ : 1;
    };
    uint64_t control_;
    Lock lock_;
  };
};
static_assert(sizeof(HashBucketOverflowEntry) == 8, "sizeof(HashBucketOverflowEntry) != 8");

/// A bucket consisting of 7 hash bucket entries, plus one hash bucket overflow entry. Fits in
/// a cache line.
struct HashBucket {
  /// Number of entries per bucket (excluding overflow entry).
  static constexpr uint32_t kNumEntries = 7;
  /// The entries.
  HashBucketEntry entries[kNumEntries];
  /// Overflow entry points to next overflow bucket, if any.
  HashBucketOverflowEntry overflow_entry;
};
static_assert(sizeof(HashBucket) == 64, "sizeof(HashBucket) != 64");

}  // namespace bfc
