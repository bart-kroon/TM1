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

#include <TMIV/Renderer/RecoverPrunedViews.h>

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <algorithm>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Renderer {
auto recoverPrunedViewAndMask(const AccessUnit &frame)
    -> pair<vector<Texture444Depth10Frame>, MaskList> {
  // Initialization
  auto prunedView = vector<Texture444Depth10Frame>{};
  auto prunedMasks = MaskList{};
  const auto &viewParamsList = frame.atlas.front().viewParamsList;

  for (const auto &viewParams : viewParamsList) {
    const auto size = viewParams.ci.projectionPlaneSize();
    prunedView.emplace_back(Texture444Frame{size.x(), size.y()}, Depth10Frame{size.x(), size.y()});
    prunedView.back().first.fillNeutral();
    prunedMasks.emplace_back(size.x(), size.y());
    prunedMasks.back().fillZero();
  }

  // Process patches
  for (const auto &atlas : frame.atlas) {
    for (const auto &patchParams : atlas.patchParamsList) {
      const auto occupancyTransform =
          OccupancyTransform{viewParamsList[patchParams.pduViewId()], patchParams};

      auto textureAtlasMap = atlas.attrFrame; // copy
      auto depthAtlasMap = atlas.geoFrame;    // copy

      auto &currentView = prunedView[patchParams.pduViewId()];
      auto &textureViewMap = currentView.first;
      auto &depthViewMap = currentView.second;

      auto &mask = prunedMasks[patchParams.pduViewId()];

      const int wP = patchParams.pdu2dSize().x();
      const int hP = patchParams.pdu2dSize().y();
      const int xP = patchParams.pdu2dPos().x();
      const int yP = patchParams.pdu2dPos().y();

      for (int dy = 0; dy < hP; dy++) {
        for (int dx = 0; dx < wP; dx++) {
          // get position
          const auto pAtlas = Vec2i{xP + dx, yP + dy};
          const auto pView = patchParams.atlasToView(pAtlas);

          // Y
          if (occupancyTransform.occupant(depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()))) {
            textureViewMap.getPlane(0)(pView.y(), pView.x()) =
                textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
            textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
          }

          // UV
          for (int p = 1; p < 3; p++) {
            if (occupancyTransform.occupant(depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()))) {
              textureViewMap.getPlane(p)(pView.y(), pView.x()) =
                  textureAtlasMap.getPlane(p)(pAtlas.y(), pAtlas.x());
              textureAtlasMap.getPlane(p)(pAtlas.y(), pAtlas.x()) = Texture444Frame::neutralColor();
            }
          }

          // Depth
          if (occupancyTransform.occupant(depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()))) {
            depthViewMap.getPlane(0)(pView.y(), pView.x()) =
                depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
            depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
            mask.getPlane(0)(pView.y(), pView.x()) = 255U;
          }
        }
      }
    }
  }

  return pair{prunedView, prunedMasks};
}
} // namespace TMIV::Renderer
