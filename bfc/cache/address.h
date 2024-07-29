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
#include <cassert>
#include <cstdint>
namespace bfc {
class Address {
 public:
  static constexpr uint64_t kAddressBits = 48;
  static constexpr uint64_t kMaxAddress = ((uint64_t)1 << kAddressBits) - 1;
  /// --of which 25 bits are used for offsets into a page, of size 2^26 = 64 MB.
  static constexpr uint64_t kOffsetBits = 26;
  static constexpr uint32_t kMaxOffset = ((uint32_t)1 << kOffsetBits) - 1;
  /// --and the remaining 22 bits are used for the page index, allowing for approximately 8 million
  /// pages.
  static constexpr uint64_t kSegmentBits = kAddressBits - kOffsetBits;
  static constexpr uint32_t kMaxSegment = ((uint32_t)1 << kSegmentBits) - 1;
  Address() { control_ = 0; }
  Address(uint32_t segment, uint32_t offset) : segment_{segment}, offset_{offset}, reserved_{0} {}
  Address(uint64_t control) : control_{control} { assert(reserved_ == 0); }
  void Clear() { control_ = 0; }
  bool operator==(const Address& addr) { return segment_ == addr.Segment() && offset_ == addr.Offset(); }

  uint64_t Control() const { return control_; }
  uint32_t Segment() const { return segment_; }
  uint32_t Offset() const { return offset_; }
  void SetSegment(uint32_t seg) { segment_ = seg; }
  void SetOffset(uint32_t offset) { offset_ = offset; }
  void IncOffset(uint32_t v) { offset_ += v; }
  // void SetInvalid(bool v) { invalid_ = v ? 1 : 0; }
  // bool IsInvalid() const { return invalid_; }
  // void SetTentative(bool v) { tentative_ = v ? 1 : 0; }
  // bool IsTentative() const { return tentative_; }
  // void SetErased(bool v) { erased_ = v ? 1 : 0; }
  // bool IsErased() { return erased_; }

 private:
  union {
    struct {
      uint64_t segment_ : kSegmentBits;
      uint64_t offset_ : kOffsetBits;          // max 64MB per segment
      uint64_t reserved_ : 64 - kAddressBits;  // 16 bits
    };
    uint64_t control_;
  };
};
static_assert(sizeof(Address) == 8, "sizeof(Address) != 8");
}  // namespace bfc