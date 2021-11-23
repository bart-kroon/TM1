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
template <typename InComponent, typename OutComponent>
void initializePrunedViewComponent(const MivBitstream::AccessUnit &inFrame,
                                   Common::V3cFrameList &outFrame, InComponent &&inComponent,
                                   OutComponent &&outComponent, uint32_t bitDepth) {
  outFrame.resize(inFrame.viewParamsList.size());

  // Scan all atlases to find the maximum needed bit depth and color format
  auto colorFormat = Common::ColorFormat::YUV400;

  for (const auto &atlas : inFrame.atlas) {
    if (atlas.asps.asps_miv_extension_present_flag() &&
        atlas.asps.asps_miv_extension().asme_ancillary_atlas_flag()) {
      continue;
    }

    const auto &in = inComponent(atlas);

    if (in.empty()) {
      continue;
    }

    bitDepth = std::max(bitDepth, in.getBitDepth());
    colorFormat = std::max(colorFormat, in.getColorFormat());
  }

  // This component is not present nor will it be generated
  if (0 == bitDepth) {
    return;
  }

  // For each view, allocate the component frame
  for (size_t viewIdx = 0; viewIdx < outFrame.size(); ++viewIdx) {
    const auto &viewParams = inFrame.viewParamsList[viewIdx];
    const auto size = Common::Vec2i{viewParams.ci.ci_projection_plane_width_minus1() + 1,
                                    viewParams.ci.ci_projection_plane_height_minus1() + 1};

    auto &out = outComponent(outFrame[viewIdx]);
    out.create(size, bitDepth, colorFormat);

    if (colorFormat != Common::ColorFormat::YUV400) {
      out.fillNeutral();
    }
  }
}

// NOTE(BK): This new implementation relies on the block to patch map. There is no assumption on
// patch ordering anymore.
auto recoverPrunedViews(const MivBitstream::AccessUnit &inFrame) -> Common::V3cFrameList {
  // Initialize
  auto outFrame = Common::V3cFrameList(inFrame.viewParamsList.size());

  initializePrunedViewComponent(
      inFrame, outFrame, [](const auto &aau) { return aau.occFrame; },
      [&](auto &frame) -> auto & { return frame.occupancy; }, 8);

  uint32_t geoBitDepth{}; // account for asme_patch_constant_depth_flag

  for (const auto &atlas : inFrame.atlas) {
    geoBitDepth = std::max(geoBitDepth, atlas.asps.asps_geometry_2d_bit_depth_minus1() + 1U);
  }

  initializePrunedViewComponent(
      inFrame, outFrame, [](const auto &aau) { return aau.geoFrame; },
      [&](auto &frame) -> auto & { return frame.geometry; }, geoBitDepth);

  initializePrunedViewComponent(
      inFrame, outFrame, [](const auto &aau) { return aau.texFrame; },
      [&](auto &frame) -> auto & { return frame.texture; }, 0);

  // For each pixel in each atlas
  for (const auto &atlas : inFrame.atlas) {
    if (atlas.asps.asps_miv_extension_present_flag() &&
        atlas.asps.asps_miv_extension().asme_ancillary_atlas_flag()) {
      continue;
    }

    for (int32_t i = 0; i < atlas.asps.asps_frame_height(); ++i) {
      for (int32_t j = 0; j < atlas.asps.asps_frame_width(); ++j) {
        // Fetch patch index
        const auto patchIdx = atlas.patchIdx(i, j);
        if (patchIdx == Common::unusedPatchIdx) {
          continue;
        }

        // Index patch and view parameters
        const auto &patchParams = atlas.patchParamsList[patchIdx];
        const auto viewIdx = inFrame.viewParamsList.indexOf(patchParams.atlasPatchProjectionId());

        // Test if this pixel is within the patch
        if (j >= patchParams.atlasPatch2dPosX() + patchParams.atlasPatch2dSizeX() ||
            i >= patchParams.atlasPatch2dPosY() + patchParams.atlasPatch2dSizeY()) {
          continue;
        }

        // Test for occupancy
        if (!atlas.occFrame.empty() && !atlas.occFrame.getPlane(0)(i, j)) {
          continue;
        }

        // Map to view position
        const auto viewPos = patchParams.atlasToView({j, i});
        const auto x = viewPos.x();
        const auto y = viewPos.y();

        // Copy occupancy
        outFrame[viewIdx].occupancy.getPlane(0)(y, x) = outFrame[viewIdx].occupancy.maxValue();

        // Copy geometry
        if (atlas.asps.asps_miv_extension_present_flag() &&
            atlas.asps.asps_miv_extension().asme_patch_constant_depth_flag()) {
          outFrame[viewIdx].geometry.getPlane(0)(y, x) =
              Common::assertDownCast<Common::DefaultElement>(patchParams.atlasPatch3dOffsetD());
        } else if (!atlas.geoFrame.empty()) {
          outFrame[viewIdx].geometry.getPlane(0)(y, x) = atlas.geoFrame.getPlane(0)(i, j);
        }

        // Copy texture
        for (int32_t d = 0; d < 3; ++d) {
          outFrame[viewIdx].texture.getPlane(d)(y, x) = atlas.texFrame.getPlane(d)(i, j);
        }
      }
    }
  }

  return outFrame;
}
} // namespace TMIV::Renderer
