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

#include <TMIV/Encoder/Encoder.h>

#include <TMIV/Common/Factory.h>

using namespace std;
using namespace TMIV::AtlasConstructor;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;
using namespace TMIV::ViewOptimizer;
using namespace TMIV::DepthOccupancy;

namespace TMIV::Encoder {
Encoder::Encoder(const Json &rootNode, const Json &componentNode) {
  m_viewOptimizer =
      Factory<IViewOptimizer>::getInstance().create("ViewOptimizer", rootNode, componentNode);
  m_atlasConstructor =
      Factory<IAtlasConstructor>::getInstance().create("AtlasConstructor", rootNode, componentNode);
  m_depthOccupancy =
      Factory<IDepthOccupancy>::getInstance().create("DepthOccupancy", rootNode, componentNode);
}

auto Encoder::prepareSequence(MivBitstream::IvSequenceParams ivSequenceParams)
    -> const MivBitstream::IvSequenceParams & {
  const auto optimal = m_viewOptimizer->optimizeSequence(move(ivSequenceParams));
  m_ivsp = &m_depthOccupancy->transformSequenceParams(
      m_atlasConstructor->prepareSequence(move(optimal.first), move(optimal.second)));
  return *m_ivsp;
}

void Encoder::prepareAccessUnit(MivBitstream::IvAccessUnitParams ivAccessUnitParams) {
  m_atlasConstructor->prepareAccessUnit(move(ivAccessUnitParams));
}

void Encoder::pushFrame(Common::MVD16Frame views) {
  return m_atlasConstructor->pushFrame(m_viewOptimizer->optimizeFrame(move(views)));
}

auto Encoder::completeAccessUnit() -> const MivBitstream::IvAccessUnitParams & {
  m_ivaup = &m_depthOccupancy->transformAccessUnitParams(m_atlasConstructor->completeAccessUnit());
  return *m_ivaup;
}

namespace {
// TODO(BK): Move geometry downscaling into a new component
auto maxPoolDownscaleGeometry(const Depth10Frame &frame, Vec2i frameSize) -> Depth10Frame {
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
          maximum = std::max(maximum, frame.getPlane(0)(i, j));
        }
      }

      result.getPlane(0)(y, x) = maximum;
    }
  }

  return frame;
}
} // namespace

auto Encoder::popAtlas() -> Common::MVD10Frame {
  auto frame = m_depthOccupancy->transformAtlases(m_atlasConstructor->popAtlas());

  // Geometry scaling
  if (m_ivsp->msp().msp_geometry_scale_enabled_flag()) {
    for (size_t atlasId = 0; atlasId < frame.size(); ++atlasId) {
      const auto &masp = m_ivaup->atlas[atlasId].asps.miv_atlas_sequence_params();
      auto frameSize = Vec2i{masp.masp_geometry_frame_width_minus1() + 1,
                             masp.masp_geometry_frame_height_minus1() + 1};
      frame[atlasId].depth = maxPoolDownscaleGeometry(frame[atlasId].depth, frameSize);
    }
  }

  return frame;
}
} // namespace TMIV::Encoder
