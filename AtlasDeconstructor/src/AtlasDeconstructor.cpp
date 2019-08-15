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

#include <TMIV/AtlasDeconstructor/AtlasDeconstructor.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Image/Image.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Image;
using namespace TMIV::Metadata;

namespace TMIV::AtlasDeconstructor {
constexpr auto neutralChroma = uint16_t(512);

AtlasDeconstructor::AtlasDeconstructor(const Common::Json & /*rootNode*/,
                                       const Common::Json & /*componentNode*/) {}

PatchIdMapList AtlasDeconstructor::getPatchIdMap(const std::vector<Vec2i> &atlasSize,
                                                 const AtlasParametersList &patchList,
                                                 const MVD10Frame &frame) {
  PatchIdMapList patchMapList;

  for (const auto &sz : atlasSize) {
    PatchIdMap patchMap(sz.x(), sz.y());
    std::fill(patchMap.getPlane(0).begin(), patchMap.getPlane(0).end(), unusedPatchId);
    patchMapList.push_back(std::move(patchMap));
  }

  for (size_t id = 0U; id < patchList.size(); ++id) {
    writePatchIdInMap(patchList[id], patchMapList, static_cast<uint16_t>(id), frame);
  }

  return patchMapList;
}

void AtlasDeconstructor::writePatchIdInMap(const AtlasParameters &patch,
                                           PatchIdMapList &patchMapList, std::uint16_t patchId,
                                           const MVD10Frame &frame) const {
  auto &patchMap = patchMapList[patch.atlasId];
  auto &depthMap = frame[patch.atlasId].second.getPlane(0);

  const Vec2i &q0 = patch.posInAtlas;
  int w = patch.patchSize.x();
  int h = patch.patchSize.y();
  bool isRotated = patch.rotation != PatchRotation::upright && patch.rotation != PatchRotation::ht;
  int xMin = q0.x();
  int xLast = q0.x() + (isRotated ? h : w);
  int yMin = q0.y();
  int yLast = q0.y() + (isRotated ? w : h);

  for (auto y = yMin; y < yLast; y++) {
    for (auto x = xMin; x < xLast; x++) {
      if (depthMap[y * patchMap.getWidth() + x] >= 64) {
        patchMap.getPlane(0)(y, x) = patchId;
      }
    }
  }
}

MVD16Frame AtlasDeconstructor::recoverPrunedView(const MVD16Frame &atlas,
                                                 const CameraParametersList &cameraList,
                                                 const AtlasParametersList &patchList) {
  // Initialization
  MVD16Frame frame;

  for (const auto &cam : cameraList) {
    TextureFrame tex(cam.size.x(), cam.size.y());

    std::fill(tex.getPlane(0).begin(), tex.getPlane(0).end(), 0);
    std::fill(tex.getPlane(1).begin(), tex.getPlane(1).end(), neutralChroma);
    std::fill(tex.getPlane(2).begin(), tex.getPlane(2).end(), neutralChroma);

    Depth16Frame depth(cam.size.x(), cam.size.y());

    std::fill(depth.getPlane(0).begin(), depth.getPlane(0).end(), 0);

    frame.push_back(TextureDepth16Frame{std::move(tex), std::move(depth)});
  }

  // Process patches
  MVD16Frame atlas_pruned = atlas;

  for (auto iter = patchList.rbegin(); iter != patchList.rend(); ++iter) {
    const auto &patch = *iter;

    auto &currentAtlas = atlas_pruned[patch.atlasId];
    auto &currentView = frame[patch.viewId];

    auto &textureAtlasMap = currentAtlas.first;
    auto &depthAtlasMap = currentAtlas.second;

    auto &textureViewMap = currentView.first;
    auto &depthViewMap = currentView.second;

    int w = patch.patchSize.x();
    int h = patch.patchSize.y();
    bool isRotated = patch.rotation == PatchRotation::ccw || patch.rotation == PatchRotation::cw;
    int wP = isRotated ? h : w;
    int hP = isRotated ? w : h;
    int xP = patch.posInAtlas.x();
    int yP = patch.posInAtlas.y();

    for (int dy = 0; dy < hP; dy++) {
      for (int dx = 0; dx < wP; dx++) {
        // get position
        Vec2i pAtlas = {xP + dx, yP + dy};
        Vec2i pView = atlasToView(pAtlas, patch);
        // Y
        if (0 < depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x())) {
          textureViewMap.getPlane(0)(pView.y(), pView.x()) =
              textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
          textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
        }
        // UV
        if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
          for (int p = 1; p < 3; p++) {
            if (0 < depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x())) {
              textureViewMap.getPlane(p)(pView.y() / 2, pView.x() / 2) =
                  textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
              textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2) = 0x8000;
            }
          }
        }
        // Depth
        if (0 < depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x())) {
          depthViewMap.getPlane(0)(pView.y(), pView.x()) =
              depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
          depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
        }
      }
    }
  }

  return frame;
}

} // namespace TMIV::AtlasDeconstructor
