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

#ifndef TMIV_AGGREGATOR_AGGREGATOR_H
#define TMIV_AGGREGATOR_AGGREGATOR_H

#include "IAggregator.h"

#include <TMIV/Common/Json.h>

namespace TMIV::Aggregator {
class Aggregator : public IAggregator {
public:
  Aggregator(const Common::Json & /*unused*/, const Common::Json & /*unused*/);
  Aggregator(const Aggregator &) = delete;
  Aggregator(Aggregator &&) = default;
  auto operator=(const Aggregator &) -> Aggregator & = delete;
  auto operator=(Aggregator &&) -> Aggregator & = default;
  ~Aggregator() override = default;

  void prepareAccessUnit() override;
  void pushMask(const Common::FrameList<uint8_t> &mask) override;
  void pushInformation(const Common::FrameList<uint32_t> &information) override;

  void completeAccessUnit() override {}
  [[nodiscard]] auto getAggregatedMask() -> Common::FrameList<uint8_t> & override;
  [[nodiscard]] auto getMeanAggregatedInformation() -> Common::FrameList<uint32_t> & override;

private:
  Common::FrameList<uint8_t> m_aggregatedMask;
  Common::FrameList<uint32_t> m_information;
  uint32_t m_frames{0};
};

inline auto Aggregator::getAggregatedMask() -> Common::FrameList<uint8_t> & {
  return m_aggregatedMask;
}
} // namespace TMIV::Aggregator

#endif
