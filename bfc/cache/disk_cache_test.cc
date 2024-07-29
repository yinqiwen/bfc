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
#include "bfc/cache/disk_cache.h"
#include <folly/Singleton.h>
#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace bfc;

TEST(DiskCacheBase, simple) {
  folly::SingletonVault::singleton()->registrationComplete();
  CacheOptions opts(100000);
  opts.dir = "./";
  using Cache = DiskCache<int64_t, std::string_view>;
  auto cache_result = Cache::New(opts);
  ASSERT_TRUE(cache_result.ok());
  auto cache = std::move(cache_result.value());
  for (int64_t i = 0; i < 100; i++) {
    std::string content = "hello, world:";
    if (i > 50) {
      content.resize(4096);
    }

    content.append(std::to_string(i));
    auto status = cache->Put(i, std::move(content));
    if (!status.ok()) {
      BFC_INFO("error:{}", status.ToString());
    }
    ASSERT_TRUE(status.ok());
  }
  for (int64_t i = 0; i < 100; i++) {
    // ReadContextPtr ctx = std::make_shared<ReadContext>();
    auto result = cache->Get(i);
    if (!result.ok()) {
      printf("###%d %s\n", i, result.status().ToString().c_str());
    }
    ASSERT_TRUE(result.ok());
    std::string content = "hello, world:";
    if (i > 50) {
      content.resize(4096);
    }
    content.append(std::to_string(i));

    ASSERT_EQ(result.value(), content);
  }
  for (int64_t i = 0; i < 100; i++) {
    // ReadContextPtr ctx = std::make_shared<ReadContext>();
    size_t n = cache->Delete(i);
    ASSERT_EQ(n, 1);
  }
}

// TEST(DiskCacheBase, reload) {
//   folly::SingletonVault::singleton()->registrationComplete();
//   CacheOptions opts(100000);
//   opts.dir = "./";
//   using Cache = DiskCache<int64_t, std::string_view>;
//   auto cache_result = Cache::New(opts);
//   ASSERT_TRUE(cache_result.ok());
//   auto cache = std::move(cache_result.value());
//   for (int64_t i = 0; i < 100; i++) {
//     std::string content = "hello, world:";
//     if (i > 50) {
//       content.resize(4096);
//     }
//     content.append(std::to_string(i));
//     auto status = cache->Put(i, std::move(content));
//     if (!status.ok()) {
//       BFC_INFO("error:{}", status.ToString());
//     }
//     ASSERT_TRUE(status.ok());
//   }
//   auto status = cache->Save();
//   ASSERT_TRUE(status.ok());
//   cache.reset();
//   opts.max_size = opts.max_size * 2;
//   cache_result = Cache::New(opts);
//   ASSERT_TRUE(cache_result.ok());
//   cache = std::move(cache_result.value());
//   for (int64_t i = 0; i < 100; i++) {
//     auto result = cache->Get(i);
//     if (!result.ok()) {
//       printf("###%d %s\n", i, result.status().ToString().c_str());
//     }
//     ASSERT_TRUE(result.ok());
//     std::string content = "hello, world:";
//     if (i > 50) {
//       content.resize(4096);
//     }
//     content.append(std::to_string(i));
//     ASSERT_EQ(result.value(), content);
//   }
// }
