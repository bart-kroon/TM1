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


#include <TMIV/Metadata/DepthOccupancyTransform.h>
#include <TMIV/Renderer/RecoverPrunedViews.h>


using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {

auto recoverPrunedViews(const Common::MVD10Frame &atlas,
                    const Metadata::ViewParamsVector &viewParamsVector,
                    const Metadata::AtlasParamsVector &atlasParamsVector)
-> Common::MVD10Frame {

  // Initialization
  MVD10Frame frame;

  for (const auto &cam : viewParamsVector) {
    TextureFrame tex(cam.size.x(), cam.size.y());
    Depth10Frame depth(cam.size.x(), cam.size.y());
    tex.fillNeutral();
    depth.fillZero(); 
    frame.push_back(TextureDepth10Frame{std::move(tex), std::move(depth)});
  }

  // Process patches
  MVD10Frame atlas_pruned = atlas;

  for (auto iter = atlasParamsVector.rbegin(); iter != atlasParamsVector.rend(); ++iter) {
    const auto &patch = *iter;
    const auto occupancyTransform = OccupancyTransform{viewParamsVector[patch.viewId], patch};

    auto &currentAtlas = atlas_pruned[patch.atlasId];
    auto &currentView = frame[patch.viewId];

    auto &textureAtlasMap = currentAtlas.first;
    auto &depthAtlasMap = currentAtlas.second;

    auto &textureViewMap = currentView.first;
    auto &depthViewMap = currentView.second;

    const auto sizeInAtlas = patch.patchSizeInAtlas();
    int wP = sizeInAtlas.x();
    int hP = sizeInAtlas.y();
    int xP = patch.posInAtlas.x();
    int yP = patch.posInAtlas.y();

    for (int dy = 0; dy < hP; dy++) {
      for (int dx = 0; dx < wP; dx++) {
        // get position
        Vec2i pAtlas = {xP + dx, yP + dy};
        Vec2i pView = atlasToView(pAtlas, patch);
        // Y
        if (occupancyTransform.occupant(depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()))) {
          textureViewMap.getPlane(0)(pView.y(), pView.x()) =
              textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
          textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
        }
        // UV
        if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
          for (int p = 1; p < 3; p++) {
            if (occupancyTransform.occupant(depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()))) {
              textureViewMap.getPlane(p)(pView.y() / 2, pView.x() / 2) =
                  textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
              textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2) = TextureFrame::neutralColor();
            }
          }
        }
        // Depth
        if (occupancyTransform.occupant(depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()))) {
          depthViewMap.getPlane(0)(pView.y(), pView.x()) =
              depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
          depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
        }
      }
    }
  }

  return frame;
}

} // namespace TMIV::Renderer
