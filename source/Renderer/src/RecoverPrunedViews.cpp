/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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
#include <iostream>

namespace TMIV::Renderer {
// NOTE(BK): This new implementation relies on the block to patch map. There is no assumption on
// patch ordering anymore.
auto recoverPrunedViewAndMask(const MivBitstream::AccessUnit &frame)
    -> std::pair<std::vector<Common::Texture444Depth10Frame>, Common::MaskList> {
  // Initialization
  auto prunedView = std::vector<Common::Texture444Depth10Frame>{};
  auto prunedMasks = Common::MaskList{};

  const auto &viewParamsList = frame.viewParamsList;

  for (const auto &viewParams : viewParamsList) {
    const auto size = viewParams.ci.projectionPlaneSize();

    // TODO(#397): What should be the bit depth of reconstructed pruned views?
    prunedView.emplace_back(Common::Texture444Frame::yuv444({size.x(), size.y()}, 10),
                            Common::Depth10Frame::lumaOnly({size.x(), size.y()}, 10));

    prunedView.back().first.fillNeutral();
    prunedMasks.emplace_back().createY(size);
  }

  // For each pixel in each atlas
  for (const auto &atlas : frame.atlas) {
    if (atlas.asps.asps_miv_extension_present_flag() &&
        atlas.asps.asps_miv_extension().asme_ancillary_atlas_flag()) {
      continue;
    }

    for (int32_t i = 0; i < atlas.asps.asps_frame_height(); ++i) {
      for (int32_t j = 0; j < atlas.asps.asps_frame_width(); ++j) {
        // Fetch patch ID
        const auto patchId = atlas.patchId(i, j);
        if (patchId == Common::unusedPatchId) {
          continue;
        }

        // Index patch and view parameters
        const auto &patchParams = atlas.patchParamsList[patchId];
        const auto viewIdx = frame.viewParamsList.indexOf(patchParams.atlasPatchProjectionId());

        // Test for occupancy
        if (!atlas.occFrame.empty() && atlas.occFrame.getPlane(0)(i, j) == 0) {
          continue;
        }

        // Map to view position
        const auto viewPos = patchParams.atlasToView({j, i});
        const auto x = viewPos.x();
        const auto y = viewPos.y();

        // temporary use only view dimensions
        if (y >= prunedView[viewIdx].first.getSize()[1] ||
            x >= prunedView[viewIdx].first.getSize()[0]) {
          continue;
        }

        // Copy geometry
        if (!atlas.geoFrame.empty() &&
            (!atlas.asps.asps_miv_extension_present_flag() ||
             !atlas.asps.asps_miv_extension().asme_patch_constant_depth_flag())) {
          prunedView[viewIdx].second.getPlane(0)(y, x) = atlas.geoFrame.getPlane(0)(i, j);
        } else if (atlas.asps.asps_miv_extension_present_flag() &&
                   atlas.asps.asps_miv_extension().asme_patch_constant_depth_flag()) {
          prunedView[viewIdx].second.getPlane(0)(y, x) =
              Common::assertDownCast<uint16_t>(patchParams.atlasPatch3dOffsetD());
        }

        // Copy attributes
        for (int32_t d = 0; d < 3; ++d) {
          prunedView[viewIdx].first.getPlane(d)(y, x) = atlas.attrFrame.getPlane(d)(i, j);
        }

        // Set mask
        prunedMasks[viewIdx].getPlane(0)(y, x) = UINT8_MAX;
      }
    }
  }

  return std::pair{prunedView, prunedMasks};
}
} // namespace TMIV::Renderer
