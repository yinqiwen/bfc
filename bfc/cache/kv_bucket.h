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
#include <stdlib.h>
#include <cstdint>
#include <utility>

namespace bfc {
template <class KeyT, class ValueT>
class KeyValueBucket {
 public:
  KeyValueBucket() {}
  ~KeyValueBucket() {
    for (size_t i = 0; i < size_; i++) {
      keys_[i].~KeyT();
      values_[i].~ValueT();
    }
    if (nullptr != keys_) {
      free(keys_);
      free(values_);
    }
  }
  uint32_t Size() const { return size_; }
  KeyT& GetKey(uint32_t i) { return keys_[i]; }
  ValueT& GetValue(uint32_t i) { return values_[i]; }
  void Add(KeyT&& key, ValueT&& val) {
    if (size_ == capacity_) {
      uint32_t new_cap = nextCapacity();
      KeyT* new_key_buf = reinterpret_cast<KeyT*>(malloc(new_cap * sizeof(KeyT)));
      ValueT* new_value_buf = reinterpret_cast<ValueT*>(malloc(new_cap * sizeof(ValueT)));
      if (nullptr != keys_) {
        for (size_t i = 0; i < size_; i++) {
          new (new_key_buf + i) KeyT(std::move(keys_[i]));
          new (new_value_buf + i) ValueT(std::move(values_[i]));
          keys_[i].~KeyT();
          values_[i].~ValueT();
        }
      }
      free(keys_);
      free(values_);
      keys_ = reinterpret_cast<KeyT*>(new_key_buf);
      values_ = reinterpret_cast<ValueT*>(new_value_buf);
      capacity_ = new_cap;
    }
    new (keys_ + size_) KeyT(std::move(key));
    new (values_ + size_) ValueT(std::move(val));
    size_++;
  }
  void Clear(uint32_t i) {
    keys_[i] = {};
    values_[i] = {};
  }
  void Remove(uint32_t i) {
    if (i < size_ - 1) {
      keys_[i] = std::move(keys_[size_ - 1]);
      values_[i] = std::move(values_[size_ - 1]);
    }
    keys_[size_ - 1].~KeyT();
    values_[size_ - 1].~ValueT();
    size_--;
  }

 private:
  static const uint32_t kMaxCapacityExtend = 4;
  uint32_t nextCapacity() const {
    if (capacity_ == 0) {
      return 1;
    }
    uint32_t nex_cap = capacity_ * 2;
    if (nex_cap > capacity_ + kMaxCapacityExtend) {
      nex_cap = capacity_ + kMaxCapacityExtend;
    }
    return nex_cap;
  }
  KeyT* keys_ = nullptr;
  ValueT* values_ = nullptr;
  uint32_t capacity_ = 0;
  uint32_t size_ = 0;
};
}  // namespace bfc