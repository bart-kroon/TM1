/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#include <TMIV/DepthOccupancy/DepthOccupancy.h>

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <iostream>
#include <stdexcept>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::DepthOccupancy {
DepthOccupancy::DepthOccupancy(uint16_t depthOccMapThresholdIfSet)
    : m_depthOccMapThresholdIfSet{depthOccMapThresholdIfSet} {
  if (depthOccMapThresholdIfSet < 1) {
    throw runtime_error("The depthOccMapThresholdIfSet parameter is only used when the encoder "
                        "needs to use occupancy. The value 0 is not allowed.");
  }
  if (depthOccMapThresholdIfSet >= 500) {
    throw runtime_error("The DepthOccupancy component takes a margin equal to the threshold, so "
                        "setting the threshold this high will make it impossible to encode depth.");
  }
}

DepthOccupancy::DepthOccupancy(const Json & /*unused*/, const Json &nodeConfig)
    : DepthOccupancy{uint16_t(nodeConfig.require("depthOccMapThresholdIfSet").asInt())} {}

auto DepthOccupancy::transformSequenceParams(MivBitstream::IvSequenceParams sequenceParams)
    -> const MivBitstream::IvSequenceParams & {
  m_inSequenceParams = move(sequenceParams);
  m_outSequenceParams = m_inSequenceParams;

  for (auto &x : m_outSequenceParams.viewParamsList) {
    if (x.hasOccupancy) {
      x.dq.dq_depth_occ_map_threshold_default(m_depthOccMapThresholdIfSet); // =T
      const auto nearLevel = 1023.F;
      const auto farLevel = float(2 * m_depthOccMapThresholdIfSet);
      // Mapping is [2T, 1023] --> [old far, near]. What is level 0? (the new far)
      x.dq.dq_norm_disp_low(x.dq.dq_norm_disp_low() +
                            (0.F - farLevel) / (nearLevel - farLevel) *
                                (x.dq.dq_norm_disp_high() - x.dq.dq_norm_disp_low()));
    }
  }

  return m_outSequenceParams;
}

auto DepthOccupancy::transformAccessUnitParams(MivBitstream::IvAccessUnitParams accessUnitParams)
    -> const MivBitstream::IvAccessUnitParams & {
  m_accessUnitParams = accessUnitParams;
  return m_accessUnitParams;
}

auto DepthOccupancy::transformAtlases(const Common::MVD16Frame &inAtlases) -> Common::MVD10Frame {
  auto outAtlases = MVD10Frame{};
  outAtlases.reserve(inAtlases.size());

  for (auto &inAtlas : inAtlases) {
    outAtlases.emplace_back(inAtlas.first,
                            Depth10Frame{inAtlas.second.getWidth(), inAtlas.second.getHeight()});
  }

  for (const auto &patch : m_accessUnitParams.patchParamsList) {
    const auto &inViewParams = m_inSequenceParams.viewParamsList[patch.pduViewId()];
    const auto &outViewParams = m_outSequenceParams.viewParamsList[patch.pduViewId()];
    const auto inOccupancyTransform = OccupancyTransform{inViewParams};
#ifndef NDEBUG
    const auto outOccupancyTransform = OccupancyTransform{outViewParams, patch};
#endif
    const auto inDepthTransform = DepthTransform<16>{inViewParams.dq};
    const auto outDepthTransform = DepthTransform<10>{outViewParams.dq, patch};

    const auto patchSizeInAtlas = patch.pdu2dSize();

    for (int i = 0; i < patchSizeInAtlas.y(); ++i) {
      for (int j = 0; j < patchSizeInAtlas.x(); ++j) {
        const int n = i + patch.pdu2dPos().y();
        const int m = j + patch.pdu2dPos().x();

        const auto inLevel = inAtlases[patch.vuhAtlasId].second.getPlane(0)(n, m);

        if (inOccupancyTransform.occupant(inLevel)) {
          const auto normDisp = inDepthTransform.expandNormDisp(inLevel);
          const auto outLevel = outDepthTransform.quantizeNormDisp(normDisp, 0);
          assert(outOccupancyTransform.occupant(outLevel));

          outAtlases[patch.vuhAtlasId].second.getPlane(0)(n, m) = outLevel;
        }
      }
    }
  }

  return outAtlases;
}
} // namespace TMIV::DepthOccupancy
