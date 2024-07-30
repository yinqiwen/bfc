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
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

namespace bfc {

// struct CacheOptions {
//   std::string dir;
//   uint32_t max_size = 4 * 1024 * 1024;
//   uint32_t max_segments = 0;  // 0 means no limit
//   uint32_t segment_delete_delay_secs = 15;
//   uint32_t save_limit_mb_bytes_per_10ms = 1;  // 100MB/s
//   uint32_t segment_ttl_secs = 24 * 3600;      // 1day
//   uint32_t sample_routine_interval_ms = 10;
//   uint32_t sample_count = 10;
//   uint32_t ttl_secs = 4 * 3600;
//   uint32_t empty_item_ttl_secs = 3600;

//   bool append_only = true;
//   CacheOptions(uint32_t n = 4 * 1024 * 1024) : max_size(n) {}
// };

struct MemCacheOptions {
  uint32_t max_size = 4 * 1024 * 1024;
  uint32_t sample_routine_interval_ms = 10;
  uint32_t sample_count = 10;
  uint32_t ttl_secs = 4 * 3600;
  uint32_t empty_item_ttl_secs = 3600;
  MemCacheOptions(uint32_t n = 4 * 1024 * 1024) : max_size(n) {}
};

struct DiskCacheOptions {
  std::string dir;
  uint32_t max_size = 4 * 1024 * 1024;
  uint32_t max_segments = 0;  // 0 means no limit
  uint32_t sample_routine_interval_ms = 10;
  uint32_t segment_delete_delay_secs = 15;
  uint32_t save_limit_mb_bytes_per_10ms = 1;  // 100MB/s
  uint32_t ttl_secs = 24 * 3600;              // 1day
  bool append_only = true;
  DiskCacheOptions(uint32_t n = 4 * 1024 * 1024) : max_size(n) {}
};

struct ReadOptions {
  bool update_access_timestamp = true;
};

}  // namespace bfc