/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TMIV_COMMON_THREAD_H
#define TMIV_COMMON_THREAD_H

#include <functional>
#include <future>
#include <vector>

namespace TMIV::Common {
inline auto threadCount() -> auto & {
  static auto singletonValue = std::thread::hardware_concurrency();

  return singletonValue;
}

inline void parallel_for(size_t nbIter, std::function<void(size_t)> fun) {
  auto segment_execute = [&](size_t first, size_t last) {
    for (auto id = first; id < last; id++) {
      fun(id);
    }
  };

  const auto chunkSize = threadCount() < nbIter ? nbIter / threadCount() : 1;
  std::vector<std::future<void>> threadList;

  for (size_t id = 0; id < nbIter; id += chunkSize) {
    threadList.push_back(std::async(segment_execute, id, std::min(id + chunkSize, nbIter)));
  }

  for (auto &thread : threadList) {
    thread.wait();
  }
}

inline void parallel_for(size_t w, size_t h, std::function<void(size_t, size_t)> fun) {
  size_t nbIter = w * h;

  if (nbIter == 0) {
    return;
  }

  auto segment_execute = [&](size_t first, size_t last) {
    size_t i0 = first / w;
    size_t i1 = std::max(i0 + 1, last / w);

    for (size_t i = i0; i < i1; i++) {
      for (size_t j = 0; j < w; j++) {
        fun(i, j);
      }
    }
  };

  std::vector<std::future<void>> threadList;
  auto chunkSize = threadCount() < nbIter ? nbIter / threadCount() : 1;

  size_t misalignment = (chunkSize % w);

  if (0 < misalignment) {
    chunkSize = chunkSize + (w - misalignment);
  }

  for (size_t id = 0; id < nbIter; id += chunkSize) {
    threadList.push_back(std::async(segment_execute, id, std::min(id + chunkSize, nbIter)));
  }

  for (auto &thread : threadList) {
    thread.wait();
  }
}
} // namespace TMIV::Common

#endif
