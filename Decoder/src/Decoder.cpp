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

#include <TMIV/Decoder/Decoder.h>

#include <TMIV/Common/Factory.h>

#include <cassert>

using namespace std;
using namespace TMIV::AtlasDeconstructor;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Renderer;

namespace TMIV::Decoder {
Decoder::Decoder(const Json &rootNode, const Json &componentNode)
    : m_atlasDeconstructor{Factory<IAtlasDeconstructor>::getInstance().create(
          "AtlasDeconstructor", rootNode, componentNode)},
      m_renderer{Factory<IRenderer>::getInstance().create("Renderer", rootNode, componentNode)} {}

void Decoder::updateSequenceParams(Metadata::IvSequenceParams ivSequenceParams) {
  m_ivSequenceParams = move(ivSequenceParams);
}

void Decoder::updateAccessUnitParams(Metadata::IvAccessUnitParams ivAccessUnitParams) {
  m_ivAccessUnitParams = move(ivAccessUnitParams);
}

void Decoder::decompressDepthRange(TextureDepth10Frame &atlas) {

  int W = atlas.second.getWidth();
  int H = atlas.second.getHeight();

  int max = 1023;
  double oldMinPercent = 1.5;

  int oldMin = max * oldMinPercent / 100;
  int newRange = max;
  int oldRange = max - oldMin;

  for (int h = 0; h < H; h++) {
    for (int w = 0; w < W; w++) {
      if (atlas.second.getPlane(0)(h, w) < oldMin) {
        atlas.second.getPlane(0)(h, w) = 0;
      } else if (atlas.second.getPlane(0)(h, w) == oldMin) {
        atlas.second.getPlane(0)(h, w) = 1;
      } else {
        atlas.second.getPlane(0)(h, w) =
            (atlas.second.getPlane(0)(h, w) - oldMin) * newRange / oldRange;
      }
    }
  }

  return;
}

auto Decoder::decodeFrame(MVD10Frame atlas, const ViewParams &target) -> Texture444Depth16Frame {

  for (int a = 0; a < atlas.size(); a++) {
    decompressDepthRange(atlas[a]);
  }
  return m_renderer->renderFrame(
      atlas, m_atlasDeconstructor->getPatchIdMap(m_ivSequenceParams, m_ivAccessUnitParams, atlas),
      m_ivSequenceParams, m_ivAccessUnitParams, target);
}

auto Decoder::getPatchIdMapList(const MVD10Frame &atlas) const -> PatchIdMapList {
  return m_atlasDeconstructor->getPatchIdMap(m_ivSequenceParams, m_ivAccessUnitParams, atlas);
}

auto Decoder::recoverPrunedView(const Common::MVD10Frame &atlas) const -> Common::MVD10Frame {
  return m_atlasDeconstructor->recoverPrunedView(atlas, m_ivSequenceParams.viewParamsList,
                                                 *m_ivAccessUnitParams.atlasParamsList);
}

} // namespace TMIV::Decoder
