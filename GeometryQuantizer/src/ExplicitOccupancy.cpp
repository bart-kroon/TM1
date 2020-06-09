/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

#include <TMIV/GeometryQuantizer/ExplicitOccupancy.h>

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <iostream>
#include <stdexcept>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::GeometryQuantizer {
ExplicitOccupancy::ExplicitOccupancy(const Json & /*unused*/, const Json & /*unused*/) {}

auto ExplicitOccupancy::transformSequenceParams(MivBitstream::IvSequenceParams sequenceParams)
    -> const MivBitstream::IvSequenceParams & {
  m_inSequenceParams = move(sequenceParams);
  m_outSequenceParams = m_inSequenceParams;
  // Always assume that first atlas is a complete one (include basic view(s)) and second atlas is an incomplete one (i.e. include patches)
  m_outSequenceParams.vps.vps_occupancy_video_present_flag(0, false);
  m_outSequenceParams.vps.vps_occupancy_video_present_flag(1, true);

  return m_outSequenceParams;
}

auto ExplicitOccupancy::transformAccessUnitParams(MivBitstream::IvAccessUnitParams accessUnitParams)
    -> const MivBitstream::IvAccessUnitParams & {
  m_accessUnitParams = accessUnitParams;
  return m_accessUnitParams;
}

auto ExplicitOccupancy::transformAtlases(const Common::MVD16Frame &inAtlases)
    -> Common::MVD10Frame {
  auto outAtlases = MVD10Frame{};
  outAtlases.reserve(inAtlases.size());

  for (const auto &inAtlas : inAtlases) {
    outAtlases.emplace_back(inAtlas.texture,
                            Depth10Frame{inAtlas.depth.getWidth(), inAtlas.depth.getHeight()},
                            inAtlas.occupancy);
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

    for (auto i = 0; i < patch.pdu2dSize().y(); ++i) {
      for (auto j = 0; j < patch.pdu2dSize().x(); ++j) {
        const auto n = i + patch.pdu2dPos().y();
        const auto m = j + patch.pdu2dPos().x();

        const auto &plane = inAtlases[patch.vuhAtlasId].depth.getPlane(0);

        if (n < 0 || n >= int(plane.height()) || m < 0 || m >= int(plane.width())) {
          abort();
        }

        const auto inLevel = plane(n, m);

        if (inOccupancyTransform.occupant(inLevel)) {
          const auto normDisp = inDepthTransform.expandNormDisp(inLevel);
          const auto outLevel = outDepthTransform.quantizeNormDisp(normDisp, 0);
          assert(outOccupancyTransform.occupant(outLevel));

          outAtlases[patch.vuhAtlasId].depth.getPlane(0)(n, m) = outLevel;
        }
      }
    }
  }

  // Apply depth padding
  padGeometryWithAvg(outAtlases);

  return outAtlases;
}

static vector<uint16_t> avg;
static bool isFirstFrame = true;
void ExplicitOccupancy::padGeometryWithAvg(MVD10Frame &atlases) {
  if (isFirstFrame)
    for (uint8_t i = 0; i < atlases.size(); ++i)
      avg.push_back(0);

  for (uint8_t i = 0; i < atlases.size(); ++i) {
    if (m_outSequenceParams.vps.vps_occupancy_video_present_flag(uint8_t(i))) {
      auto &depthAtlasMap = atlases[i].depth;
      if (isFirstFrame) {
        // Find Average geometry value per atlas at first frame in IRAP only
        double sum = 0, count = 0;
        for (int y = 0; y < depthAtlasMap.getHeight(); y++) {
          for (int x = 0; x < depthAtlasMap.getWidth(); x++) {
            auto depth = depthAtlasMap.getPlane(0)(y, x);
            if (depth > 0) {
              sum = sum + (double)depth;
              count++;
            }
          }
        }
        avg[i] = sum / count;
        cout << "Sum = " << sum << ", count = " << count << endl;
        cout << "Padded depth value for group's atlas " << (int)i << " is " << avg[i] << endl;
      }
      for (int y = 0; y < depthAtlasMap.getHeight(); y++) {
        for (int x = 0; x < depthAtlasMap.getWidth(); x++) {
          auto depth = depthAtlasMap.getPlane(0)(y, x);
          if (depth == 0) {
            depthAtlasMap.getPlane(0)(y, x) = avg[i];
          }
        }
      }
    }
  }
  isFirstFrame = false;
}

} // namespace TMIV::DepthOccupancy
