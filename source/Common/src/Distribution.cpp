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

#include <TMIV/Common/Distribution.h>

#include <TMIV/Common/verify.h>

#include <cmath>
#include <numeric>

namespace TMIV::Common {
auto Distribution::count() const -> size_t {
  return std::accumulate(m_counts.cbegin(), m_counts.cend(), size_t{},
                         [](size_t init, const auto &kvp) { return init + kvp.second; });
}

auto Distribution::icdf(float p) const -> float {
  PRECONDITION(0.F <= p && p <= 1.F && *this);

  const auto target = static_cast<double>(count()) * p;
  auto subCount = size_t{};
  auto lastValue = std::numeric_limits<float>::quiet_NaN();

  for (const auto &[value, count] : m_counts) {
    if (!std::isnan(lastValue)) {
      return 0.5F * (lastValue + value);
    }
    if (subCount += count; target < static_cast<double>(subCount)) {
      return value;
    }
    if (target == static_cast<double>(subCount)) {
      lastValue = value;
    }
  }
  return lastValue;
}

auto Distribution::sample(float value) -> Distribution & {
  PRECONDITION(std::isfinite(value));
  ++m_counts.emplace(value, size_t{}).first->second;
  return *this;
}
} // namespace TMIV::Common
