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
#include "bfc/common/rate_limit_file.h"
#include <thread>
namespace bfc {
RateLimitFile::RateLimitFile(int fd, uint32_t limit_bytes_per_10ms)
    : fd_(fd), limiter_(getMaxPerInterval(limit_bytes_per_10ms), std::chrono::milliseconds(10)) {}

ssize_t RateLimitFile::WriteFull(const void* buf, size_t count) {
  ssize_t writed_n = 0;
  const uint8_t* wbuf = reinterpret_cast<const uint8_t*>(buf);
  while (writed_n < static_cast<ssize_t>(count)) {
    while (!limiter_.check()) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    size_t to_write_n = kWriteBufferSize;
    if (writed_n + to_write_n > count) {
      to_write_n = count - writed_n;
    }
    ssize_t n = folly::writeFull(fd_, wbuf + writed_n, to_write_n);
    if (n != static_cast<ssize_t>(to_write_n)) {
      if (n < 0) {
        return n;
      }
      return n + writed_n;
    }
    writed_n += to_write_n;
  }
  return writed_n;
}
}  // namespace bfc