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
#include <folly/Singleton.h>
#include <cstdint>
#include <string>
#include "bfc/cache/mem_cache.h"
#include "bfc/common/time_helper.h"
#include "folly/Random.h"
#include "folly/ThreadCachedInt.h"
#include "gflags/gflags.h"

DEFINE_uint32(max_size, 5000000, "max cache size");
DEFINE_uint32(write_threads, 1, "write threads");
DEFINE_uint32(read_threads, 4, "read threads");
DEFINE_uint32(N, 1000000, "test count");

using IntCache = bfc::MemCache<int64_t, std::string>;
using StringCache = bfc::MemCache<std::string, std::string>;
static constexpr std::string_view g_val_prefix = "test_value:";
static constexpr std::string_view g_string_key_prefix = "test_key:";
static folly::ThreadLocalPRNG g_rng;

static folly::ThreadCachedInt<uint64_t> g_write_fail{0};
static folly::ThreadCachedInt<uint64_t> g_read_fail{0};
static folly::ThreadCachedInt<uint64_t> g_read_latency{0};
static folly::ThreadCachedInt<uint64_t> g_write_latency{0};

template <typename T>
T rand_key();
template <>
int64_t rand_key<int64_t>() {
  auto rand = folly::Random::rand32(0, FLAGS_max_size * 2, g_rng);
  return static_cast<int64_t>(rand);
}
template <>
std::string rand_key<std::string>() {
  auto rand = folly::Random::rand32(0, FLAGS_max_size * 2, g_rng);
  std::string key(g_string_key_prefix);
  key.append(std::to_string(rand));
  return key;
}

template <typename T>
T new_key(uint32_t i);
template <>
int64_t new_key<int64_t>(uint32_t i) {
  return static_cast<int64_t>(i);
}
template <>
std::string new_key<std::string>(uint32_t i) {
  std::string key(g_string_key_prefix);
  key.append(std::to_string(i));
  return key;
}

template <typename Cache>
struct TestCache {
  std::unique_ptr<Cache> cache_;
  TestCache(std::unique_ptr<Cache>&& c) : cache_(std::move(c)) {}
  void Fill() {
    auto start = bfc::gettimeofday_us();
    for (uint32_t i = 0; i < FLAGS_max_size; i++) {
      auto key = new_key<typename Cache::key_type>(i);
      std::string val(g_val_prefix);
      val.append(std::to_string(key));
      auto _ = cache_->Put(key, std::move(val));
    }
    auto latency_us = bfc::gettimeofday_us() - start;
    float qps = FLAGS_max_size * 1000000.0 / latency_us;
    fmt::print("[Fill]Cost {}us to insert {} items in ONE thread, avg qps:{}\n", latency_us, FLAGS_max_size, qps);
  }
  void Read() {
    auto start = bfc::gettimeofday_us();
    for (uint32_t i = 0; i < FLAGS_N; i++) {
      auto key = rand_key<typename Cache::key_type>();
      auto result = cache_->Get(key);
      if (result.ok()) {
        std::string val(g_val_prefix);
        val.append(std::to_string(key));
        if (result.value() != val) {
          g_read_fail.increment(1);
        }
      }
    }
    auto latency_us = bfc::gettimeofday_us() - start;
    g_read_latency.increment(latency_us);
  }
  void Write() {
    auto start = bfc::gettimeofday_us();
    for (uint32_t i = 0; i < FLAGS_N; i++) {
      auto key = rand_key<typename Cache::key_type>();
      std::string value(g_val_prefix);
      value.append(std::to_string(key));
      auto status = cache_->Put(key, std::move(value));
      if (!status.ok()) {
        g_write_fail.increment(1);
      }
    }
    auto latency_us = bfc::gettimeofday_us() - start;
    g_write_latency.increment(latency_us);
  }
};

template <typename Cache>
void test_cache() {
  typename Cache::Options opt;
  opt.max_size = FLAGS_max_size;
  auto result = Cache::New(opt);
  if (!result.ok()) {
    fmt::print("###Error:{}\n", result.status().ToString());
    return;
  }
  TestCache<Cache> test_cache(std::move(result.value()));
  test_cache.Fill();

  std::vector<std::thread> read_threads;
  std::vector<std::thread> write_threads;
  for (uint32_t i = 0; i < FLAGS_write_threads; i++) {
    write_threads.emplace_back(std::thread(std::bind(&TestCache<Cache>::Write, &test_cache)));
  }
  for (uint32_t i = 0; i < FLAGS_read_threads; i++) {
    read_threads.emplace_back(std::thread(std::bind(&TestCache<Cache>::Read, &test_cache)));
  }
  for (auto& t : write_threads) {
    t.join();
  }
  for (auto& t : read_threads) {
    t.join();
  }
  float read_qps = FLAGS_read_threads * FLAGS_N * 1000000.0 / g_read_latency.readFull();
  float write_qps = FLAGS_read_threads * FLAGS_N * 1000000.0 / g_write_latency.readFull();
  bfc::Stats& mem_cache_stats = test_cache.cache_->GetStats();
  fmt::print("[Read]Read {} items per thread({}), avg qps per thread:{}, cache_hit:{}, cache_missing:{}, evict:{}\n",
             FLAGS_N, FLAGS_read_threads, read_qps, mem_cache_stats.cache_hit.readFull(),
             mem_cache_stats.cache_missing.readFull(), mem_cache_stats.evit_count.readFull());
  fmt::print("[Write]Write {} items per thread({}), avg qps per thread:{}, write_fail:{}\n", FLAGS_N,
             FLAGS_write_threads, write_qps, g_write_fail.readFull());
}

int main(int argc, char** argv) {
  google::ParseCommandLineNonHelpFlags(&argc, &argv, false);
  folly::SingletonVault::singleton()->registrationComplete();
  fmt::print("============Int Cache Benchmark=============\n");
  test_cache<IntCache>();
  fmt::print("============String Cache Benchmark=============\n");
  return 0;
}
