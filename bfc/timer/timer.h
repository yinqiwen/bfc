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

#pragma once
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <thread>

#include "folly/executors/CPUThreadPoolExecutor.h"
#include "folly/io/async/EventBase.h"
#include "folly/io/async/HHWheelTimer.h"

namespace bfc {

using TimerTask = std::function<void(void)>;

// struct State {
//   std::atomic<bool> state{false};
//   ~State() { printf("####state dest\n"); }
// };
class Timer;
class TimerTaskId {
 public:
  using State = std::atomic<bool>;
  using StatePtr = std::shared_ptr<State>;
  TimerTaskId() {
    // cancel_ = std::make_shared<std::atomic<bool>>(false);
    cancel_ = std::make_shared<State>();
  }
  bool IsCanceled() const { return cancel_->load(); }
  void Cancel() {
    // cancel_->store(true);
    cancel_->store(true);
  }
  ~TimerTaskId() {}

 private:
  // std::shared_ptr<std::atomic<bool>> cancel_;
  StatePtr cancel_;

  friend class Timer;
};

class Timer {
 public:
  Timer();
  ~Timer();
  static std::shared_ptr<Timer> GetInstance();
  template <class Duration>
  TimerTaskId Schedule(TimerTask&& task, Duration delay) {
    std::chrono::milliseconds delay_mills = std::chrono::duration_cast<std::chrono::milliseconds>(delay);
    return DoSchedule(std::move(task), delay_mills);
  }

  template <class Duration>
  TimerTaskId ScheduleAtFixedRate(TimerTask&& task, Duration delay) {
    std::chrono::milliseconds delay_mills = std::chrono::duration_cast<std::chrono::milliseconds>(delay);
    return DoScheduleAtFixedRate(std::move(task), delay_mills);
  }

 private:
  struct Task : folly::HHWheelTimer::Callback {
    folly::HHWheelTimer& timer;
    folly::CPUThreadPoolExecutor* executor = nullptr;
    TimerTask f;
    TimerTaskId id;
    std::chrono::milliseconds delay;
    bool fix_rate = false;
    explicit Task(folly::HHWheelTimer& t, folly::CPUThreadPoolExecutor* exec, TimerTask&& func,
                  std::chrono::milliseconds d, bool fix)
        : timer(t), executor(exec), f(std::move(func)), delay(d), fix_rate(fix) {}
    void timeoutExpired() noexcept override;
  };

  TimerTaskId DoSchedule(TimerTask&& task, std::chrono::milliseconds delay);
  TimerTaskId DoScheduleAtFixedRate(TimerTask&& task, std::chrono::milliseconds delay);
  void EventLoop();

  folly::EventBase event_base_;
  std::thread event_base_thread_;
  std::unique_ptr<folly::CPUThreadPoolExecutor> executor_;
};
}  // namespace bfc