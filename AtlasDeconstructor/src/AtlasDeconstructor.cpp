/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

using namespace std;
using namespace TMIV::Common;

namespace TMIV::AtlasDeconstructor {

AtlasDeconstructor::AtlasDeconstructor(const Common::Json &) {}

PatchIdMapList
AtlasDeconstructor::getPatchIdMap(const std::vector<Vec2i> &atlasSize,
                                  const PatchParameterList &patchList) {
  PatchIdMapList patchMapList;

  for (const auto &sz : atlasSize) {
    PatchIdMap patchMap(sz.x(), sz.y());
    std::fill(patchMap.getPlane(0).begin(), patchMap.getPlane(0).end(),
              unusedPatchId);
    patchMapList.push_back(std::move(patchMap));
  }

  for (auto id = 0u; id < patchList.size(); id++)
    writePatchIdInMap(patchList[id], patchMapList, static_cast<uint16_t>(id));

  return patchMapList;
}

void AtlasDeconstructor::writePatchIdInMap(const PatchParameters &patch,
                                           PatchIdMapList &patchMapList,
                                           std::uint16_t patchId) const {
  auto &patchMap = patchMapList[patch.atlasId];

  const Vec2i &q0 = patch.posInAtlas;
  int w = patch.patchSize.x(), h = patch.patchSize.y();
  bool isRotated = (patch.rotation != Metadata::PatchRotation::upright);
  int xMin = q0.x(), xLast = q0.x() + (isRotated ? h : w);
  int yMin = q0.y(), yLast = q0.y() + (isRotated ? w : h);

  for (auto y = yMin; y < yLast; y++)
    std::fill(patchMap.getPlane(0).row_begin(y) + xMin,
              patchMap.getPlane(0).row_begin(y) + xLast, patchId);
}

MVD16Frame
AtlasDeconstructor::recoverPrunedView(const MVD10Frame &atlas,
                                      const CameraParameterList &cameraList,
                                      const PatchParameterList &patchList) {
  // Initialization
  MVD10Frame mvd10;

  for (const auto &cam : cameraList) {
    TextureFrame tex(cam.size.x(), cam.size.y());

    std::fill(tex.getPlane(0).begin(), tex.getPlane(0).end(), 0);
    std::fill(tex.getPlane(1).begin(), tex.getPlane(1).end(), 512);
    std::fill(tex.getPlane(2).begin(), tex.getPlane(2).end(), 512);

    Depth10Frame depth(cam.size.x(), cam.size.y());

    std::fill(depth.getPlane(0).begin(), depth.getPlane(0).end(), 0);

    mvd10.push_back(TextureDepth10Frame{std::move(tex), std::move(depth)});
  }

  // Process patches
  MVD10Frame atlas_pruned = atlas;

  for (auto iter = patchList.rbegin(); iter != patchList.rend(); ++iter) {
    const auto &patch = *iter;

    auto &currentAtlas = atlas_pruned[patch.atlasId];
    auto &currentView = mvd10[patch.viewId];

    auto &textureAtlasMap = currentAtlas.first;
    auto &depthAtlasMap = currentAtlas.second;

    auto &textureViewMap = currentView.first;
    auto &depthViewMap = currentView.second;

    int w = patch.patchSize.x(), h = patch.patchSize.y();
    int xM = patch.posInView.x(), yM = patch.posInView.y();
    int xP = patch.posInAtlas.x(), yP = patch.posInAtlas.y();
    int w_tex = ((xM + w) <= (int)textureViewMap.getWidth())
                    ? w
                    : ((int)textureViewMap.getWidth() - xM);
    int h_tex = ((yM + h) <= (int)textureViewMap.getHeight())
                    ? h
                    : ((int)textureViewMap.getHeight() - yM);

    if (patch.rotation == Metadata::PatchRotation::upright) {
      for (int dy = 0; dy < h_tex; dy++) {
        // Y
        std::copy(textureAtlasMap.getPlane(0).row_begin(yP + dy) + xP,
                  textureAtlasMap.getPlane(0).row_begin(yP + dy) + (xP + w_tex),
                  textureViewMap.getPlane(0).row_begin(yM + dy) + xM);

        std::fill(textureAtlasMap.getPlane(0).row_begin(yP + dy) + xP,
                  textureAtlasMap.getPlane(0).row_begin(yP + dy) + (xP + w), 0);

        // UV
        if ((dy % 2) == 0) {
          for (int p = 1; p < 3; p++) {
            std::copy(
                textureAtlasMap.getPlane(p).row_begin((yP + dy) / 2) + xP / 2,
                textureAtlasMap.getPlane(p).row_begin((yP + dy) / 2) +
                    ((xP + w_tex) / 2),
                textureViewMap.getPlane(p).row_begin((yM + dy) / 2) + xM / 2);

            std::fill(textureAtlasMap.getPlane(p).row_begin((yP + dy) / 2) +
                          xP / 2,
                      textureAtlasMap.getPlane(p).row_begin((yP + dy) / 2) +
                          ((xP + w) / 2),
                      512);
          }
        }

        // Depth
        std::copy(depthAtlasMap.getPlane(0).row_begin(yP + dy) + xP,
                  depthAtlasMap.getPlane(0).row_begin(yP + dy) + (xP + w_tex),
                  depthViewMap.getPlane(0).row_begin(yM + dy) + xM);

        std::fill(depthAtlasMap.getPlane(0).row_begin(yP + dy) + xP,
                  depthAtlasMap.getPlane(0).row_begin(yP + dy) + (xP + w), 0);
      }
    } else {
      for (int dy = 0; dy < h_tex; dy++) {
        // Y
        std::copy(
            std::make_reverse_iterator(
                textureAtlasMap.getPlane(0).col_begin(xP + dy) + (yP + w)),
            std::make_reverse_iterator(
                textureAtlasMap.getPlane(0).col_begin(xP + dy) +
                (yP + (w - w_tex))),
            textureViewMap.getPlane(0).row_begin(yM + dy) + xM);

        std::fill(
            std::make_reverse_iterator(
                textureAtlasMap.getPlane(0).col_begin(xP + dy) + (yP + w)),
            std::make_reverse_iterator(
                textureAtlasMap.getPlane(0).col_begin(xP + dy) + yP),
            0);

        // UV
        if ((dy % 2) == 0) {
          for (int p = 1; p < 3; p++) {
            std::copy(std::make_reverse_iterator(
                          textureAtlasMap.getPlane(p).col_begin((xP + dy) / 2) +
                          (yP + w) / 2),
                      std::make_reverse_iterator(
                          textureAtlasMap.getPlane(p).col_begin((xP + dy) / 2) +
                          (yP + (w - w_tex)) / 2),
                      textureViewMap.getPlane(p).row_begin((yM + dy) / 2) +
                          xM / 2);

            std::fill(std::make_reverse_iterator(
                          textureAtlasMap.getPlane(p).col_begin((xP + dy) / 2) +
                          (yP + w) / 2),
                      std::make_reverse_iterator(
                          textureAtlasMap.getPlane(p).col_begin((xP + dy) / 2) +
                          yP / 2),
                      512);
          }
        }

        // Depth
        std::copy(std::make_reverse_iterator(
                      depthAtlasMap.getPlane(0).col_begin(xP + dy) + (yP + w)),
                  std::make_reverse_iterator(
                      depthAtlasMap.getPlane(0).col_begin(xP + dy) +
                      (yP + (w - w_tex))),
                  depthViewMap.getPlane(0).row_begin(yM + dy) + xM);

        std::fill(std::make_reverse_iterator(
                      depthAtlasMap.getPlane(0).col_begin(xP + dy) + (yP + w)),
                  std::make_reverse_iterator(
                      depthAtlasMap.getPlane(0).col_begin(xP + dy) + yP),
                  0);
      }
    }
  }

  // Conversion
  MVD16Frame mvd16;

  for (auto view : mvd10) {
    Depth16Frame depth16(view.second.getWidth(), view.second.getHeight());
    convert(view.second, depth16);
    mvd16.push_back(
        TextureDepth16Frame(std::move(view.first), std::move(depth16)));
  }

  return mvd16;
}

} // namespace TMIV::AtlasDeconstructor
