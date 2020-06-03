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

#include <TMIV/MivBitstream/IvSequenceParams.h>

#include <TMIV/MivBitstream/verify.h>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
IvSequenceParams::IvSequenceParams() : IvSequenceParams{false} {}

IvSequenceParams::IvSequenceParams(bool haveTexture)
    : IvSequenceParams{SizeVector{{0xFFFF, 0xFFFF}}, haveTexture} {}

IvSequenceParams::IvSequenceParams(const SizeVector &atlasSizes, bool haveTexture) {
  vps.profile_tier_level()
      .ptl_level_idc(PtlLevelIdc::Level_3_0)
      .ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc::HEVC_Main10)
      .ptl_profile_pcc_toolset_idc(PtlProfilePccToolsetIdc::MIV_Main)
      .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::MIV_Main);

  VERIFY_MIVBITSTREAM(!atlasSizes.empty());
  vps.vps_atlas_count_minus1(uint8_t(atlasSizes.size() - 1));

  for (size_t atlasId = 0; atlasId < atlasSizes.size(); ++atlasId) {
    const auto a = uint8_t(atlasId);
    vps.vps_atlas_id(a, a)
        .vps_frame_width(a, atlasSizes[atlasId].x())
        .vps_frame_height(a, atlasSizes[atlasId].y())
        .vps_geometry_video_present_flag(a, true)
        .vps_attribute_video_present_flag(a, haveTexture);

    vps.geometry_information(a).gi_geometry_nominal_2d_bitdepth_minus1(9);

    if (haveTexture) {
      vps.attribute_information(a)
          .ai_attribute_count(1)
          .ai_attribute_type_id(0, AiAttributeTypeId::ATTR_TEXTURE)
          .ai_attribute_dimension_minus1(0, 2)
          .ai_attribute_nominal_2d_bitdepth_minus1(0, 9);
    }
  }
}

void IvSequenceParams::updateMvpl() {
  auto &x = this->mvpl();

  VERIFY_MIVBITSTREAM(!viewParamsList.empty());
  x.mvp_num_views_minus1(uint16_t(viewParamsList.size() - 1));
  x.mvp_intrinsic_params_equal_flag(
      all_of(viewParamsList.begin(), viewParamsList.end(),
             [this](const auto &x) { return x.ci == viewParamsList.front().ci; }));
  x.mvp_depth_quantization_params_equal_flag(
      all_of(viewParamsList.begin(), viewParamsList.end(),
             [this](const auto &x) { return x.dq == viewParamsList.front().dq; }));
  x.mvp_pruning_graph_params_present_flag(viewParamsList.front().pp.has_value());

  for (uint16_t i = 0; i <= x.mvp_num_views_minus1(); ++i) {
    const auto &vp = viewParamsList[i];
    x.camera_extrinsics(i) = vp.ce;

    if (i == 0 || !x.mvp_intrinsic_params_equal_flag()) {
      x.camera_intrinsics(i) = vp.ci;
    }
    if (i == 0 || !x.mvp_depth_quantization_params_equal_flag()) {
      x.depth_quantization(i) = vp.dq;
    }
    VERIFY_MIVBITSTREAM(vp.pp.has_value() == x.mvp_pruning_graph_params_present_flag());
    if (vp.pp.has_value()) {
      x.pruning_parent(i) = *vp.pp;
    }
  }
}

auto IvSequenceParams::vme() const noexcept -> const VpsMivExtension & {
  return vps.vps_miv_extension();
}

auto IvSequenceParams::vme() noexcept -> VpsMivExtension & {
  return vps.vps_extension_present_flag(true).vps_miv_extension_flag(true).vps_miv_extension();
}

auto IvSequenceParams::aame() const noexcept -> const AapsMivExtension & {
  return aaps.aaps_miv_extension();
}

auto IvSequenceParams::aame() noexcept -> AapsMivExtension & {
  return aaps.aaps_extension_present_flag(true).aaps_miv_extension_flag(true).aaps_miv_extension();
}

auto IvSequenceParams::mvpl() const noexcept -> const MivViewParamsList & {
  return aame().miv_view_params_list();
}

auto IvSequenceParams::mvpl() noexcept -> MivViewParamsList & {
  return aame()
      .aame_miv_view_params_list_update_mode(MvpUpdateMode::VPL_INITLIST)
      .miv_view_params_list();
}

auto IvSequenceParams::operator==(const IvSequenceParams &other) const -> bool {
  return vps == other.vps && aaps == other.aaps && viewingSpace == other.viewingSpace;
}
} // namespace TMIV::MivBitstream
