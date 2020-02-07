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

#include <TMIV/Renderer/SubBlockCuller.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/Metadata/DepthOccupancyTransform.h>
#include <cassert>
#include <iostream>

#include "TMIV/Renderer/Engine.h"
#include <TMIV/Renderer/Synthesizer.h>
#include <TMIV/Renderer/reprojectPoints.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::Renderer {

 SubBlockCuller::SubBlockCuller(const Json & /*rootNode*/, const Json & /*componentNode*/) {}

static auto affineParameterList(const ViewParamsVector &viewParamsVector,
                                const ViewParams &target) {
  vector<pair<Mat3x3f, Vec3f>> result;
  result.reserve(viewParamsVector.size());
  transform(
      begin(viewParamsVector), end(viewParamsVector), back_inserter(result),
      [&target](const ViewParams &viewParams) { return affineParameters(viewParams, target); });
  return result;
}

 bool choosePatch(const AtlasParameters &patch, const ViewParamsVector &cameras,
                 const ViewParams &target) {
  const auto &camera = cameras[patch.viewId];
  auto R_t = affineParameterList(cameras, target);
  const auto &R = R_t[patch.viewId].first;
  const auto &t = R_t[patch.viewId].second;

  Vec2f uv[4];
  Vec2f xy_v[8];
  float w = patch.patchSizeInView.x(), h = patch.patchSizeInView.y();
  uv[0] = Vec2f(patch.posInView);
  uv[1] = uv[0] + Vec2f{w, 0};
  uv[2] = uv[0] + Vec2f{0, h};
  uv[3] = uv[0] + Vec2f{w, h};

  // Using Camera depth
  double patch_dep_near = 0;
  double patch_dep_far = 0;
  if (camera.normDispRange.x()<=0) {
    patch_dep_far = 1000.0;
  } else {
    patch_dep_far = 1 / camera.normDispRange.x();
  }
  if (!camera.normDispRange.y()) {
    patch_dep_near = 1000.0;
  } else {
    patch_dep_near = 1 / camera.normDispRange.y();
  }

  // Using Patch depth
  // const auto &dep_far = patch.depthStart;
  // const auto &dep_near = 0;
  /*double temp1 = camera.normDispRange.y() - camera.normDispRange.x();
  double temp2 = camera.normDispRange.x();
  double temp3 = n / 65535 * temp1 + temp2;
  double patch_near = 1 / temp3;
  temp3 = f / 65535 * temp1 + temp2;
  double patch_far = 1 / temp3;*/

  for (int i = 0; i < 4; i++) {
    const auto xyz = R * unprojectVertex(uv[i], patch_dep_near, camera) + t;
    const auto rayAngle = angle(xyz, xyz - t);
    SceneVertexDescriptor v;
    v.position = xyz;
    v.rayAngle = rayAngle;
    auto pix = visit(
        [&](auto const &x) {
          Engine<std::decay_t<decltype(x)>> engine{target};
          return engine.projectVertex(v);
        },
        target.projection);
    xy_v[i] = pix.position;
  }
  for (int i = 0; i < 4; i++) {
    const auto xyz = R * unprojectVertex(uv[i], patch_dep_far, camera) + t;
    const auto rayAngle = angle(xyz, xyz - t);
    SceneVertexDescriptor v;
    v.position = xyz;
    v.rayAngle = rayAngle;
    auto pix = visit(
        [&](auto const &x) {
          Engine<std::decay_t<decltype(x)>> engine{target};
          return engine.projectVertex(v);
        },
        target.projection);
    // wangbin modify
    xy_v[i + 4] = pix.position;
  }
  float xy_v_xmax = -1000;
  float xy_v_xmin = 65536;
  float xy_v_ymax = -1000;
  float xy_v_ymin = 65536;
  for (int i = 0; i < 8; i++) {
    if (xy_v[i][0] > xy_v_xmax)
      xy_v_xmax = xy_v[i][0];
    if (xy_v[i][0] < xy_v_xmin)
      xy_v_xmin = xy_v[i][0];
    if (xy_v[i][1] > xy_v_ymax)
      xy_v_ymax = xy_v[i][1];
    if (xy_v[i][1] < xy_v_ymin)
      xy_v_ymin = xy_v[i][1];
  }
  if (xy_v_xmin > target.size.x() || xy_v_xmax < 0 || xy_v_ymax < 0 ||
      xy_v_ymin > target.size.y() ||
      (xy_v_xmin != xy_v_xmin && xy_v_xmax != xy_v_xmax && xy_v_ymin != xy_v_ymin &&
       xy_v_ymax != xy_v_ymax))
    return 0;
  else
    return 1;
}

 auto baseview_divide(const AtlasParameters &patch, Vec2i blocksizes) {

  int blocknums_w = patch.patchSizeInView.x() / blocksizes.x();
  int blocknums_h = patch.patchSizeInView.y() / blocksizes.y();
  int blocknums_all = blocknums_w * blocknums_h;
  AtlasParamsVector subblock(blocknums_all, patch);
  for (int i = 0; i < blocknums_h; i++) {
    for (int j = 0; j < blocknums_w; j++) {
      subblock[i * blocknums_w + j].patchSizeInView.x() = blocksizes.x();
      subblock[i * blocknums_w + j].patchSizeInView.y() = blocksizes.y();
      subblock[i * blocknums_w + j].posInView.x() = patch.posInView.x() + j * blocksizes.x();
      subblock[i * blocknums_w + j].posInView.y() = patch.posInView.y() + i * blocksizes.y();
      subblock[i * blocknums_w + j].posInAtlas.x() = patch.posInAtlas.x() + j * blocksizes.x();
      subblock[i * blocknums_w + j].posInAtlas.y() = patch.posInAtlas.x() + i * blocksizes.y();
    }
  }
  return subblock;
}

 auto SubBlockCuller::updatePatchIdmap(const MVD10Frame &atlas, const PatchIdMapList &maps,
                                      const IvSequenceParams &ivSequenceParams,
                                      const IvAccessUnitParams &ivAccessUnitParams,
                                      const ViewParams &target) -> PatchIdMapList {
  PatchIdMapList updatedpatchMapList = move(maps);
  const auto &viewParamsList = ivSequenceParams.viewParamsList;
  const auto &atlasParamsList = *ivAccessUnitParams.atlasParamsList;


  for (size_t id = 0U; id < atlasParamsList.size(); ++id) {
    // If patch is as large as source view
    if (atlasParamsList[id].patchSizeInView.x() ==
            viewParamsList[atlasParamsList[id].viewId].size.x() &&
        atlasParamsList[id].patchSizeInView.y() ==
            viewParamsList[atlasParamsList[id].viewId].size.y()) {

      // size of sub-block is fixed now.
      Vec2i blocksizes = {128, 128};
      AtlasParamsVector blocks = baseview_divide(atlasParamsList[id], blocksizes);
      for (int i = 0; i < blocks.size(); i++) {
        if (!choosePatch(blocks[i], viewParamsList, target)) {
          erasePatchIdInMap(blocks[i], updatedpatchMapList, static_cast<uint16_t>(id));
        }
      }
    } else {
      if (!choosePatch(atlasParamsList[id], viewParamsList, target)) {
        erasePatchIdInMap(atlasParamsList[id], updatedpatchMapList, static_cast<uint16_t>(id));
      }
    }
  }
  return updatedpatchMapList;
}

 void SubBlockCuller::erasePatchIdInMap(const AtlasParameters &patch,
                                           PatchIdMapList &patchMapList, uint16_t patchId) {
  auto &patchMap = patchMapList[patch.atlasId];

  const Vec2i &q0 = patch.posInAtlas;
  const auto sizeInAtlas = patch.patchSizeInAtlas();
  int xMin = q0.x();
  int xLast = q0.x() + sizeInAtlas.x();
  int yMin = q0.y();
  int yLast = q0.y() + sizeInAtlas.y();


  for (auto y = yMin; y < yLast; y++) {
    for (auto x = xMin; x < xLast; x++) {
      if (patchMap.getPlane(0)(y, x) == patchId) {
        patchMap.getPlane(0)(y, x) = unusedPatchId;
      }
    }
  }
}

} // namespace TMIV::Renderer
