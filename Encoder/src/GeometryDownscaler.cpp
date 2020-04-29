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

#include <TMIV/Encoder/GeometryDownscaler.h>

#include <algorithm>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Encoder {
GeometryDownscaler::GeometryDownscaler(const Json &rootNode, const Json & /* componentNode */)
    : m_geometryScaleEnabledFlag{rootNode.require("geometryScaleEnabledFlag").asBool()} {}

auto GeometryDownscaler::transformSequenceParams(IvSequenceParams ivSequenceParams)
    -> const IvSequenceParams & {
  m_ivSequenceParams = move(ivSequenceParams);

  if (m_geometryScaleEnabledFlag) {
    m_ivSequenceParams.vps.vps_miv_extension_flag(true);
    m_ivSequenceParams.msp().msp_geometry_scale_enabled_flag(true);
  }

  return m_ivSequenceParams;
}

auto GeometryDownscaler::transformAccessUnitParams(IvAccessUnitParams ivAccessUnitParams)
    -> const IvAccessUnitParams & {
  m_ivAccessUnitParams = move(ivAccessUnitParams);

  if (m_geometryScaleEnabledFlag) {
    for (auto &atlas : m_ivAccessUnitParams.atlas) {
      atlas.asps.asps_miv_extension_present_flag(true);
      atlas.asps.miv_atlas_sequence_params()
          .masp_geometry_frame_width_minus1(atlas.asps.asps_frame_width() / 2 - 1)
          .masp_geometry_frame_height_minus1(atlas.asps.asps_frame_height() / 2 - 1);
    }
  }

  return m_ivAccessUnitParams;
}

namespace {
auto maxPool(const Depth10Frame &frame, Vec2i frameSize) -> Depth10Frame {
  auto result = Depth10Frame{frameSize.x(), frameSize.y()};

  for (int y = 0; y < frameSize.y(); ++y) {
    const int i1 = y * frame.getHeight() / frameSize.y();
    const int i2 = (y + 1) * frame.getHeight() / frameSize.y();

    for (int x = 0; x < frameSize.x(); ++x) {
      const int j1 = x * frame.getWidth() / frameSize.x();
      const int j2 = (x + 1) * frame.getWidth() / frameSize.x();

      auto maximum = uint16_t{};

      for (int i = i1; i < i2; ++i) {
        for (int j = j1; j < j2; ++j) {
          maximum = max(maximum, frame.getPlane(0)(i, j));
        }
      }

      result.getPlane(0)(y, x) = maximum;
    }
  }

  return result;
}
} // namespace

auto GeometryDownscaler::transformFrame(MVD10Frame frame) -> MVD10Frame {
  if (m_ivSequenceParams.msp().msp_geometry_scale_enabled_flag()) {
    for (size_t atlasId = 0; atlasId < frame.size(); ++atlasId) {
      const auto &masp = m_ivAccessUnitParams.atlas[atlasId].asps.miv_atlas_sequence_params();
      const auto frameSize = Vec2i{masp.masp_geometry_frame_width_minus1() + 1,
                                   masp.masp_geometry_frame_height_minus1() + 1};
      frame[atlasId].depth = maxPool(frame[atlasId].depth, frameSize);
    }
  }
  return frame;
}
} // namespace TMIV::Encoder
