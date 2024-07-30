/*
 *Copyright (c) 2013-2013, yinqiwen <yinqiwen@gmail.com>
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
 *  * Neither the name of Redis nor the names of its contributors may be used
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

#include "bfc/common/time_helper.h"

#include <limits.h>
#include <sys/time.h>
#include <time.h>

namespace bfc {

int64_t gettimeofday_us() {
  struct timeval tv;
  uint64_t ust;
  gettimeofday(&tv, nullptr);
  ust = ((int64_t)tv.tv_sec) * 1000000;
  ust += tv.tv_usec;
  return ust;
}
int64_t gettimeofday_ms() { return gettimeofday_us() / 1000; }
int64_t gettimeofday_s() {
  int64_t secs = gettimeofday_us() / 1000000;
  return secs;
}
uint32_t gettimeofday_minutes() { return static_cast<uint32_t>(gettimeofday_us() / 60000000); }
int64_t unixsec_from_timefield(const std::string& time_field) {
  // const char *time_details = "16:35:12";
  struct tm tm;
  strptime(time_field.c_str(), "%Y-%m-%d %H:%M:%S", &tm);
  return fast_mktime(&tm);
}
time_t fast_mktime(struct tm* tm) {
  static struct tm cache = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  static time_t time_cache = 0;
  // static time_t (*mktime_real)(struct tm *tm) = NULL;
  time_t result;
  time_t hmsarg;

  /* the epoch time portion of the request */
  hmsarg = 3600 * tm->tm_hour + 60 * tm->tm_min + tm->tm_sec;

  if (cache.tm_mday == tm->tm_mday && cache.tm_mon == tm->tm_mon && cache.tm_year == tm->tm_year) {
    /* cached - just add h,m,s from request to midnight */
    result = time_cache + hmsarg;

    /* Obscure, but documented, return value: only this value in arg struct.
     *
     * BUG: dst switchover was computed by mktime_real() for time 00:00:00
     * of arg day. So this return value WILL be wrong for switchover days
     * after the switchover occurs.  There is no clean way to detect this
     * situation in stock glibc.  This bug will be reflected in unit test
     * until fixed.  See also github issues #1 and #2.
     */
    tm->tm_isdst = cache.tm_isdst;
  } else {
    /* not cached - recompute midnight on requested day */
    cache.tm_mday = tm->tm_mday;
    cache.tm_mon = tm->tm_mon;
    cache.tm_year = tm->tm_year;
    time_cache = mktime(&cache);
    tm->tm_isdst = cache.tm_isdst;
    result = (-1 == time_cache) ? -1 : time_cache + hmsarg;
  }

  return result;
}
}  // namespace bfc
