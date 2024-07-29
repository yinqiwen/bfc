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

#include "folly/Singleton.h"
#include "gflags/gflags.h"

DEFINE_int32(bfc_timer_executor_thread_num, 2, "cpu timer executor thread num");

namespace {
struct PrivateTag {};
}  // namespace
namespace bfc {
static folly::Singleton<Timer, PrivateTag> the_singleton([]() { return new Timer; });

std::shared_ptr<Timer> Timer::GetInstance() { return the_singleton.try_get(); }

void Timer::Task::timeoutExpired() noexcept {
  if (id.IsCanceled()) {
    delete this;
    return;
  }
  auto exec_f = f;
  executor->add(std::move(exec_f));
  if (fix_rate) {
    timer.scheduleTimeout(this, delay);
  } else {
    delete this;
  }
}

Timer::Timer() : event_base_thread_(&Timer::EventLoop, this) {
  event_base_.waitUntilRunning();
  event_base_.runInEventBaseThread([this] { event_base_.setName("HHWheelTimer"); });

  executor_ = std::make_unique<folly::CPUThreadPoolExecutor>(
      FLAGS_bfc_timer_executor_thread_num, std::make_shared<folly::NamedThreadFactory>("bfc_timer_executor"));
}

void Timer::EventLoop() { event_base_.loopForever(); }

TimerTaskId Timer::DoSchedule(TimerTask&& task, std::chrono::milliseconds delay) {
  Task* t = new Task(event_base_.timer(), executor_.get(), std::move(task), delay, false);
  auto id = t->id;
  event_base_.runInEventBaseThread([this, t]() {
    folly::HHWheelTimer& timer = event_base_.timer();
    timer.scheduleTimeout(t, t->delay);
  });

  return id;
}
TimerTaskId Timer::DoScheduleAtFixedRate(TimerTask&& task, std::chrono::milliseconds delay) {
  Task* t = new Task(event_base_.timer(), executor_.get(), std::move(task), delay, true);
  auto id = t->id;
  event_base_.runInEventBaseThread([this, t]() {
    folly::HHWheelTimer& timer = event_base_.timer();
    timer.scheduleTimeout(t, t->delay);
  });

  return id;
}
Timer::~Timer() {
  event_base_.runInEventBaseThreadAndWait([this] {
    folly::HHWheelTimer& t = event_base_.timer();
    t.cancelAll();
    event_base_.terminateLoopSoon();
  });
  event_base_thread_.join();
}
}  // namespace bfc
