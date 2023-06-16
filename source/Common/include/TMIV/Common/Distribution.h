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

#ifndef TMIV_COMMON_DISTRIBUTION_H
#define TMIV_COMMON_DISTRIBUTION_H

#include <cstddef>
#include <map>

namespace TMIV::Common {
// Measure the distribution of a stream of samples
class Distribution {
public:
  // One or more samples?
  [[nodiscard]] explicit operator bool() const { return !m_counts.empty(); }

  // Number of unique samples
  [[nodiscard]] auto size() const { return m_counts.size(); }

  // Number of samples
  [[nodiscard]] auto count() const -> size_t;

  // Inverse cummulative distribution function maps [0, 1] to normalized disparity
  //
  // Preconditions: p in [0, 1] and one or more samples
  [[nodiscard]] auto icdf(float p) const -> float;

  // Add a sample to the geometry distribution
  //
  // Precondition: finite value
  auto sample(float value) -> Distribution &;

private:
  std::map<float,  // sample value
           size_t> // sample count
      m_counts;
};

// Return the median value of the distribution
//
// Preconditions:
//   * one or more samples
//   * With an odd number of samples returns the middle one
//   * With an even number of samples averages the middle two
[[nodiscard]] inline auto median(const Distribution &dist) { return dist.icdf(0.5F); }

// Return the minimum vlaue of the distribution
//
// Precondition: one or more samples
[[nodiscard]] inline auto min(const Distribution &dist) { return dist.icdf(0.F); }

// Return the maximum value of the distribution
//
// Precondition: one or more samples
[[nodiscard]] inline auto max(const Distribution &dist) { return dist.icdf(1.F); }

} // namespace TMIV::Common

#endif
