/*
 *Copyright (c) 2021, qiyingwang <qiyingwang@tencent.com>
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of rimos nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 *BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "bfc/timer/timer.h"
#include <chrono>
#include <string>

#include "folly/Singleton.h"
#include "gtest/gtest.h"
using namespace std::chrono_literals;

namespace bfc {
namespace testing {
TEST(TimerTest, Sched1) {
  folly::SingletonVault::singleton()->registrationComplete();
  std::shared_ptr<Timer> timer = Timer::GetInstance();
  // EXPECT_TRUE(timer != nullptr);
  int count = 0;
  auto id = timer->Schedule([&count]() { count = 1; }, std::chrono::seconds(1));
  std::this_thread::sleep_for(2s);
  EXPECT_EQ(count, 1);

  count = 0;
  auto id1 = timer->Schedule([&count]() { count = 1; }, std::chrono::seconds(1));
  id1.Cancel();
  std::this_thread::sleep_for(2s);
  EXPECT_EQ(count, 0);
}

TEST(TimerTest, Sched2) {
  folly::SingletonVault::singleton()->registrationComplete();
  std::shared_ptr<Timer> timer = Timer::GetInstance();
  EXPECT_TRUE(timer != nullptr);
  std::this_thread::sleep_for(1s);
  int count = 0;
  auto id = timer->ScheduleAtFixedRate([&count]() { count++; }, std::chrono::seconds(1));
  std::this_thread::sleep_for(2500ms);
  id.Cancel();
  EXPECT_EQ(count, 2);

  count = 0;
  auto id1 = timer->ScheduleAtFixedRate([&count]() { count++; }, std::chrono::seconds(1));
  id1.Cancel();
  std::this_thread::sleep_for(2500ms);
  EXPECT_EQ(count, 0);
}
}  // namespace testing
}  // namespace bfc
