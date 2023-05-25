/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <TMIV/MivBitstream/AccessUnit.h>

#include <TMIV/MivBitstream/Formatters.h>

namespace TMIV::MivBitstream {
auto AccessUnit::sequenceConfig() const -> SequenceConfig {
  auto x = SequenceConfig{};

  x.contentName = "decoded";

  if (vui && vui->vui_timing_info_present_flag()) {
    x.frameRate = static_cast<double>(vui->vui_time_scale()) /
                  static_cast<double>(vui->vui_num_units_in_tick());
  }

  x.cameras.resize(viewParamsList.size());

  std::transform(viewParamsList.cbegin(), viewParamsList.cend(), x.cameras.begin(),
                 [this](const ViewParams &vp_) {
                   auto c = CameraConfig{};
                   c.viewParams = vp_;

                   for (const auto &a : atlas) {
                     if (!a.texFrame.empty()) {
                       c.bitDepthTexture = std::max(c.bitDepthTexture, a.texFrame.getBitDepth());
                     }

                     if (!a.geoFrame.empty()) {
                       c.bitDepthGeometry = std::max(c.bitDepthGeometry, a.geoFrame.getBitDepth());
                     }
                   }

                   return c;
                 });

  x.boundingBoxCenter =
      std::accumulate(viewParamsList.cbegin(), viewParamsList.cend(), Common::Vec3d{},
                      [Z = 1. / static_cast<double>(viewParamsList.size())](
                          const Common::Vec3d &init, const ViewParams &vp_) {
                        return init + Z * Common::Vec3d{vp_.pose.position};
                      });

  std::transform(viewParamsList.cbegin(), viewParamsList.cend(),
                 std::inserter(x.sourceCameraNames, x.sourceCameraNames.end()),
                 [](const auto &vp_) {
                   if (vp_.name.empty()) {
                     throw std::runtime_error("The decoder needs to assign view names");
                   }
                   return vp_.name;
                 });

  return x;
}

void requireAllPatchesWithinProjectionPlaneBounds(const ViewParamsList &vpl,
                                                  const PatchParamsList &ppl) {
  auto patchIdx = 0;

  for (const auto &pp : ppl) {
    const auto &vp = vpl[pp.atlasPatchProjectionId()];

    const auto size_u = vp.ci.ci_projection_plane_width_minus1() + 1;
    const auto size_v = vp.ci.ci_projection_plane_height_minus1() + 1;

    const auto u_1 = pp.atlasPatch3dOffsetU();
    const auto v_1 = pp.atlasPatch3dOffsetV();
    const auto u_2 = u_1 + pp.atlasPatch3dSizeU();
    const auto v_2 = v_1 + pp.atlasPatch3dSizeV();

    if (u_1 < 0 || u_1 > u_2 || u_2 > size_u || v_1 < 0 || v_1 > v_2 || v_2 > size_v) {
      throw std::runtime_error(
          fmt::format("Patch with index {} and projection ID {} is out of projection plane bounds",
                      patchIdx, pp.atlasPatchProjectionId()));
    }

    ++patchIdx;
  }
}

void requireAllPatchesWithinAtlasFrameBounds(const PatchParamsList &ppl,
                                             const AtlasSequenceParameterSetRBSP &asps) {
  auto patchIdx = 0;

  for (const auto &pp : ppl) {
    VERIFY(0 <= pp.atlasPatch2dSizeX());
    VERIFY(0 <= pp.atlasPatch2dSizeY());

    const auto size_x = asps.asps_frame_width();
    const auto size_y = asps.asps_frame_height();

    const auto x_1 = pp.atlasPatch2dPosX();
    const auto y_1 = pp.atlasPatch2dPosY();
    const auto x_2 = x_1 + pp.atlasPatch2dSizeX();
    const auto y_2 = y_1 + pp.atlasPatch2dSizeY();

    if (x_1 < 0 || x_2 > size_x || y_1 < 0 || y_2 > size_y) {
      throw std::runtime_error(
          fmt::format("Patch with index {} and projection ID {} is out of atlas frame bounds",
                      patchIdx, pp.atlasPatchProjectionId()));
    }
  }
}
} // namespace TMIV::MivBitstream
