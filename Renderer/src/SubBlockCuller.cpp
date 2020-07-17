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

#include <TMIV/Renderer/SubBlockCuller.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <cassert>
#include <iostream>

#include "TMIV/Renderer/Engine.h"
#include <TMIV/Renderer/reprojectPoints.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Renderer {
SubBlockCuller::SubBlockCuller(const Json & /*rootNode*/, const Json & /*componentNode*/) {}

auto choosePatch(const PatchParams &patch, const ViewParamsList &cameras, const ViewParams &target)
    -> bool {
  const auto &camera = cameras[patch.pduViewIdx()];
  const auto R_t = AffineTransform(cameras[patch.pduViewIdx()].ce, target.ce);

  auto uv = array<Vec2f, 4>{};
  auto xy_v = array<Vec2f, 8>{};
  const auto w = static_cast<float>(patch.pduViewSize().x());
  const auto h = static_cast<float>(patch.pduViewSize().y());
  uv[0] = Vec2f(patch.pduViewPos());
  uv[1] = uv[0] + Vec2f{w, 0};
  uv[2] = uv[0] + Vec2f{0, h};
  uv[3] = uv[0] + Vec2f{w, h};

  // Using Camera depth
  const auto patch_dep_near =
      1.F / max(MivBitstream::impl::minNormDisp, camera.dq.dq_norm_disp_low());
  const auto patch_dep_far =
      1.F / max(MivBitstream::impl::minNormDisp, camera.dq.dq_norm_disp_high());

  for (int i = 0; i < 4; i++) {
    const auto xyz = R_t(unprojectVertex(uv[i], patch_dep_near, camera.ci));
    const auto rayAngle = angle(xyz, xyz - R_t.translation());
    SceneVertexDescriptor v;
    v.position = xyz;
    v.rayAngle = rayAngle;
    auto pix = target.ci.dispatch([&](auto camType) {
      Engine<camType> engine{target.ci};
      return engine.projectVertex(v);
    });
    xy_v[i] = pix.position;
  }
  for (int i = 0; i < 4; i++) {
    const auto xyz = R_t(unprojectVertex(uv[i], patch_dep_far, camera.ci));
    const auto rayAngle = angle(xyz, xyz - R_t.translation());
    SceneVertexDescriptor v;
    v.position = xyz;
    v.rayAngle = rayAngle;
    auto pix = target.ci.dispatch([&](auto camType) {
      Engine<camType> engine{target.ci};
      return engine.projectVertex(v);
    });
    // wangbin modify
    xy_v[i + 4] = pix.position;
  }
  float xy_v_xmax = -1000;
  float xy_v_xmin = 65536;
  float xy_v_ymax = -1000;
  float xy_v_ymin = 65536;
  for (auto &i : xy_v) {
    if (i[0] > xy_v_xmax) {
      xy_v_xmax = i[0];
    }
    if (i[0] < xy_v_xmin) {
      xy_v_xmin = i[0];
    }
    if (i[1] > xy_v_ymax) {
      xy_v_ymax = i[1];
    }
    if (i[1] < xy_v_ymin) {
      xy_v_ymin = i[1];
    }
  }
  return !(xy_v_xmin > target.ci.projectionPlaneSize().x() || xy_v_xmax < 0 || xy_v_ymax < 0 ||
           xy_v_ymin > target.ci.projectionPlaneSize().y() ||
           (xy_v_xmin != xy_v_xmin && xy_v_xmax != xy_v_xmax && xy_v_ymin != xy_v_ymin &&
            xy_v_ymax != xy_v_ymax));
}

auto divideInBlocks(const PatchParams &patch, Vec2i blockSize) {
  assert(patch.pduOrientationIndex() == FlexiblePatchOrientation::FPO_NULL);

  int blocknums_w = patch.pduViewSize().x() / blockSize.x();
  int blocknums_h = patch.pduViewSize().y() / blockSize.y();
  int blocknums_all = blocknums_w * blocknums_h;
  PatchParamsList subblock(blocknums_all, patch);

  for (int i = 0; i < blocknums_h; i++) {
    for (int j = 0; j < blocknums_w; j++) {
      const auto offset = Vec2i{j * blockSize.x(), i * blockSize.y()};

      auto &b = subblock[i * blocknums_w + j];
      b.pduViewSize(blockSize);
      b.pduViewPos(b.pduViewPos() + offset);
      b.pdu2dPos(b.pdu2dPos() + offset);
    }
  }
  return subblock;
}

auto SubBlockCuller::filterBlockToPatchMap(const Decoder::AccessUnit &frame,
                                           const Decoder::AtlasAccessUnit &atlas,
                                           const ViewParams &viewportParams) const
    -> BlockToPatchMap {
  auto result = atlas.blockToPatchMap;

  for (size_t patchIdx = 0; patchIdx < atlas.patchParamsList.size(); ++patchIdx) {
    const auto &patch = atlas.patchParamsList[patchIdx];
    const auto &view = frame.viewParamsList[patch.pduViewIdx()];

    if (patch.pduViewSize() == view.ci.projectionPlaneSize()) {
      // The size of the sub-block is fixed for now
      const auto blockSize = Vec2i{128, 128};

      for (const auto &block : divideInBlocks(patch, blockSize)) {
        if (!choosePatch(block, frame.viewParamsList, viewportParams)) {
          inplaceErasePatch(result, block, uint16_t(patchIdx), atlas.asps);
        }
      }
    } else {
      if (!choosePatch(patch, frame.viewParamsList, viewportParams)) {
        inplaceErasePatch(result, patch, uint16_t(patchIdx), atlas.asps);
      }
    }
  }
  return result;
}

void SubBlockCuller::inplaceErasePatch(BlockToPatchMap &patchMap, const PatchParams &patch,
                                       uint16_t patchId,
                                       const AtlasSequenceParameterSetRBSP &asps) {
  const auto n = 1 << asps.asps_log2_patch_packing_block_size();
  const auto first = patch.pdu2dPos() / n;
  const auto last = first + patch.pdu2dSize() / n;

  for (auto y = first.y(); y < last.y(); ++y) {
    for (auto x = first.x(); x < last.x(); ++x) {
      if (patchMap.getPlane(0)(y, x) == patchId) {
        patchMap.getPlane(0)(y, x) = unusedPatchId;
      }
    }
  }
}

} // namespace TMIV::Renderer
