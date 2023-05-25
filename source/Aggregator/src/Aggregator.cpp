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

#include <TMIV/Aggregator/Aggregator.h>

namespace TMIV::Aggregator {
Aggregator::Aggregator(const Common::Json & /*rootNode*/, const Common::Json & /*componentNode*/) {}

void Aggregator::prepareAccessUnit() {
  m_frames = 0;
  m_aggregatedMask.clear();
  m_information.clear();
}

void Aggregator::pushMask(const Common::FrameList<uint8_t> &mask) {
  if (m_aggregatedMask.empty()) {
    m_aggregatedMask = mask;
  } else {
    for (size_t i = 0; i < mask.size(); i++) {
      std::transform(m_aggregatedMask[i].getPlane(0).begin(), m_aggregatedMask[i].getPlane(0).end(),
                     mask[i].getPlane(0).begin(), m_aggregatedMask[i].getPlane(0).begin(),
                     [](uint8_t v1, uint8_t v2) { return std::max(v1, v2); });
    }
  }
}

void Aggregator::pushInformation(const Common::FrameList<uint32_t> &information) {
  m_frames += 1;
  if (m_information.empty()) {
    m_information = information;
  } else {
    for (size_t i = 0; i < m_information.size(); i++) {
      std::transform(m_information[i].getPlane(0).begin(), m_information[i].getPlane(0).end(),
                     information[i].getPlane(0).begin(), m_information[i].getPlane(0).begin(),
                     [](uint16_t v1, uint16_t v2) { return v1 + v2; });
    }
  }
}

auto Aggregator::getMeanAggregatedInformation() -> Common::FrameList<uint32_t> & {
  for (auto &informaiton : m_information) {
    for (auto &value : informaiton.getPlanes()[0]) {
      value = value / m_frames;
    }
  }
  return m_information;
}
} // namespace TMIV::Aggregator
