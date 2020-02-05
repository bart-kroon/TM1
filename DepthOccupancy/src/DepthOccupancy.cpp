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

#include <TMIV/Metadata/DepthOccupancyTransform.h>

#include <iostream>
#include <stdexcept>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::DepthOccupancy {
DepthOccupancy::DepthOccupancy(uint16_t depthOccMapThreshold)
    : m_depthOccMapThreshold{depthOccMapThreshold} {
  if (depthOccMapThreshold < 1) {
    throw runtime_error("The depthOccMapThreshold parameter is only used when a view has invalid "
                        "depth. The value 0 is not allowed.");
  }
  if (depthOccMapThreshold >= 500) {
    throw runtime_error("The DepthOccupancy component takes a margin equal to the threshold, so "
                        "setting the threshold this high will make it impossible to encode depth.");
  }
}

DepthOccupancy::DepthOccupancy(const Json & /*unused*/, const Json &nodeConfig)
    : DepthOccupancy{uint16_t(nodeConfig.require("depthOccMapThreshold").asInt())} {}

auto DepthOccupancy::transformSequenceParams(Metadata::IvSequenceParams sequenceParams)
    -> const Metadata::IvSequenceParams & {
  m_inSequenceParams = move(sequenceParams);
  m_outSequenceParams = m_inSequenceParams;

  for (auto &x : m_outSequenceParams.viewParamsList) {
    if (x.depthOccMapThreshold != 0 || m_depthOccMapThreshold != 0) {
      x.depthOccMapThreshold = std::max(x.depthOccMapThreshold, m_depthOccMapThreshold); // =T
      const auto nearLevel = 1023.F;
      const auto farLevel = float(2 * x.depthOccMapThreshold);
      // Mapping is [2T, 1023] --> [old far, near]. What is level 0? (the new far)
      x.normDispRange[0] +=
          (0.F - farLevel) / (nearLevel - farLevel) * (x.normDispRange[1] - x.normDispRange[0]);
    }
  }

  return m_outSequenceParams;
}

auto DepthOccupancy::transformAccessUnitParams(Metadata::IvAccessUnitParams accessUnitParams)
    -> const Metadata::IvAccessUnitParams & {
  m_accessUnitParams = accessUnitParams;
  if (m_accessUnitParams.atlasParamsList) {
    m_accessUnitParams.atlasParamsList->depthOccupancyParamsPresentFlags =
        vector<bool>(m_accessUnitParams.atlasParamsList->atlasSizes.size(), false);
  }
  return m_accessUnitParams;
}

auto DepthOccupancy::transformAtlases(const Common::MVD16Frame &inAtlases) -> Common::MVD10Frame {
  auto outAtlases = MVD10Frame{};
  outAtlases.reserve(inAtlases.size());

  for (auto &inAtlas : inAtlases) {
    outAtlases.emplace_back(inAtlas.first,
                            Depth10Frame{inAtlas.second.getWidth(), inAtlas.second.getHeight()});
  }

  for (const auto &patch : *m_accessUnitParams.atlasParamsList) {
    const auto &inViewParams = m_inSequenceParams.viewParamsList[patch.viewId];
    const auto &outViewParams = m_outSequenceParams.viewParamsList[patch.viewId];
    const auto inOccupancyTransform = OccupancyTransform{inViewParams};
#ifndef NDEBUG
    const auto outOccupancyTransform = OccupancyTransform{outViewParams, patch};
#endif
    const auto inDepthTransform = DepthTransform<16>{inViewParams};
    const auto outDepthTransform = DepthTransform<10>{outViewParams, patch};

    const auto patchSizeInAtlas = patch.patchSizeInAtlas();

    for (int i = 0; i < patchSizeInAtlas.y(); ++i) {
      for (int j = 0; j < patchSizeInAtlas.x(); ++j) {
        const int n = i + patch.posInAtlas.y();
        const int m = j + patch.posInAtlas.x();

        const auto inLevel = inAtlases[patch.atlasId].second.getPlane(0)(n, m);

        if (inOccupancyTransform.occupant(inLevel)) {
          const auto normDisp = inDepthTransform.expandNormDisp(inLevel);
          const auto outLevel = outDepthTransform.quantizeNormDisp(normDisp, 0);
          assert(outOccupancyTransform.occupant(outLevel));

          outAtlases[patch.atlasId].second.getPlane(0)(n, m) = outLevel;
        }
      }
    }
  }

  return outAtlases;
}
} // namespace TMIV::DepthOccupancy
