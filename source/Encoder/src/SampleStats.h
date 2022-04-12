/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#ifndef TMIV_ENCODER_SAMPLE_STATS_H
#define TMIV_ENCODER_SAMPLE_STATS_H

#include <algorithm>

namespace TMIV::Encoder {
struct SampleStats {
  [[nodiscard]] constexpr auto min() const noexcept { return m_min; }
  [[nodiscard]] constexpr auto max() const noexcept { return m_max; }
  [[nodiscard]] constexpr auto sum() const noexcept { return m_sum; }
  [[nodiscard]] constexpr auto count() const noexcept { return m_count; }

  [[nodiscard]] constexpr auto floorMean() const noexcept {
    return 0 == m_count ? 0U : m_sum / m_count;
  }

  constexpr auto operator<<(int64_t sample) noexcept -> auto & {
    m_min = std::min(m_min, sample);
    m_max = std::max(m_max, sample);
    m_sum += sample;
    ++m_count;
    return *this;
  }

  constexpr auto operator+=(const SampleStats &rhs) noexcept -> auto & {
    m_min = std::min(m_min, rhs.m_min);
    m_max = std::max(m_max, rhs.m_max);
    m_sum += rhs.m_sum;
    m_count += rhs.m_count;
    return *this;
  }

  [[nodiscard]] constexpr auto operator+(const SampleStats &rhs) const noexcept {
    return SampleStats{*this} += rhs;
  }

private:
  int64_t m_min{INT64_MAX};
  int64_t m_max{INT64_MIN};
  int64_t m_sum{};
  int64_t m_count{};
};
} // namespace TMIV::Encoder

#endif
