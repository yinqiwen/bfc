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

#pragma once
#include <memory>
#include <string>

#include "absl/status/statusor.h"

namespace bfc {
class MmapFile {
 public:
  static absl::StatusOr<std::unique_ptr<MmapFile>> Open(const std::string& path, void* fixed_addr, size_t size,
                                                        int prot, int flags, bool own);

  template <typename T>
  inline T* As() {
    return reinterpret_cast<T*>(mapping_addr_);
  }
  size_t Size() const { return size_; }

  ~MmapFile();

 private:
  MmapFile();
  absl::Status Init(const std::string& path, void* fixed_addr, size_t size, int prot, int flags, bool own);
  void* mapping_addr_ = nullptr;
  size_t size_ = 0;
  bool own_ = false;
};
}  // namespace bfc