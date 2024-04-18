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

#include <algorithm>
#include <functional>
#include <future>
#include <vector>

namespace TMIV::Common {
inline auto threadCount() -> unsigned & {
  static unsigned singletonValue = std::thread::hardware_concurrency();

  return singletonValue;
}

template <typename Fun, typename = std::enable_if_t<std::is_invocable_v<Fun, size_t>>>
void parallelFor(size_t size, Fun fun) {
  PRECONDITION(0 < threadCount());

  auto threads = std::vector<std::thread>(std::min(size, size_t{threadCount()}));

  for (size_t k = 0; k < threads.size(); ++k) {
    threads[k] = std::thread{[&fun](size_t first, size_t last) {
                               for (size_t i = first; i < last; ++i) {
                                 fun(i);
                               }
                             },
                             k * size / threads.size(), (k + 1) * size / threads.size()};
  }

  for (auto &thread : threads) {
    thread.join();
  }
}

template <typename Fun, typename = std::enable_if_t<std::is_invocable_v<Fun, size_t, size_t>>>
void parallelFor(size_t columns, size_t rows, Fun fun) {
  parallelFor(rows, [columns, &fun](size_t i) {
    for (size_t j = 0; j < columns; ++j) {
      fun(i, j);
    }
  });
}
} // namespace TMIV::Common

#endif
