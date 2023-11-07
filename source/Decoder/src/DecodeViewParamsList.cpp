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

#include <TMIV/Decoder/DecodeViewParamsList.h>

#include <fmt/format.h>

namespace TMIV::Decoder {
namespace {
void decodeMvpl(const MivBitstream::MivViewParamsList &mvpl, bool dqParamsPresentFlag,
                bool csParamsPresentFlag, MivBitstream::ViewParamsList &vpl) {
  vpl.assign(mvpl.mvp_num_views_minus1() + size_t{1}, {});

  for (uint16_t viewIdx = 0; viewIdx <= mvpl.mvp_num_views_minus1(); ++viewIdx) {
    auto &vp = vpl[viewIdx];
    vp.viewId = mvpl.mvp_view_id(viewIdx);
    vp.pose = MivBitstream::Pose::decodeFrom(mvpl.camera_extrinsics(viewIdx));
    vp.viewInpaintFlag = mvpl.mvp_inpaint_flag(viewIdx);
    vp.backgroundViewFlag = mvpl.mvp_view_background_flag(viewIdx);
    vp.ci = mvpl.camera_intrinsics(viewIdx);
    if (dqParamsPresentFlag) {
      vp.dq = mvpl.depth_quantization(viewIdx);
    }
    if (csParamsPresentFlag) {
      vp.cs = mvpl.chroma_scaling(viewIdx);
    }
    if (mvpl.mvp_pruning_graph_params_present_flag()) {
      vp.pp = mvpl.pruning_parent(viewIdx);
    }

    vp.name = fmt::format("pv{:02}", viewIdx);
  }
  vpl.constructViewIdIndex();
}

void decodeMvpue(const MivBitstream::MivViewParamsUpdateExtrinsics &mvpue,
                 MivBitstream::ViewParamsList &vpl) {
  for (uint16_t i = 0; i <= mvpue.mvpue_num_view_updates_minus1(); ++i) {
    vpl[mvpue.mvpue_view_idx(i)].pose = MivBitstream::Pose::decodeFrom(mvpue.camera_extrinsics(i));
  }
}

void decodeMvpui(const MivBitstream::MivViewParamsUpdateIntrinsics &mvpui,
                 MivBitstream::ViewParamsList &vpl) {
  for (uint16_t i = 0; i <= mvpui.mvpui_num_view_updates_minus1(); ++i) {
    vpl[mvpui.mvpui_view_idx(i)].ci = mvpui.camera_intrinsics(i);
  }
}

void decodeMvpudq(const MivBitstream::MivViewParamsUpdateDepthQuantization &mvpudq,
                  MivBitstream::ViewParamsList &vpl) {
  for (uint16_t i = 0; i <= mvpudq.mvpudq_num_view_updates_minus1(); ++i) {
    vpl[mvpudq.mvpudq_view_idx(i)].dq = mvpudq.depth_quantization(i);
  }
}

void decodeMvpucs(const MivBitstream::MivViewParamsUpdateChromaScaling &mvpucs,
                  MivBitstream::ViewParamsList &vpl) {
  for (uint16_t i = 0; i <= mvpucs.mvpucs_num_view_updates_minus1(); ++i) {
    vpl[mvpucs.mvpucs_view_idx(i)].cs = mvpucs.chroma_scaling(i);
  }
}
} // namespace

void decodeViewParamsList(const CommonAtlasAccessUnit &au, MivBitstream::ViewParamsList &vpl) {
  if (au.caf.caf_extension_present_flag() && au.caf.caf_miv_extension_present_flag()) {
    const auto &came = au.caf.caf_miv_extension();
    const auto casme_depth_quantization_params_present_flag =
        au.casps.casps_miv_extension_present_flag() &&
        au.casps.casps_miv_extension().casme_depth_quantization_params_present_flag();
    const auto casme_chroma_scaling_present_flag =
        au.casps.casps_miv_2_extension_present_flag() &&
        au.casps.casps_miv_2_extension().casme_chroma_scaling_present_flag();

    if (au.foc == 0) {
      decodeMvpl(came.miv_view_params_list(), casme_depth_quantization_params_present_flag,
                 casme_chroma_scaling_present_flag, vpl);
    } else {
      if (came.came_update_extrinsics_flag()) {
        decodeMvpue(came.miv_view_params_update_extrinsics(), vpl);
      }
      if (came.came_update_intrinsics_flag()) {
        decodeMvpui(came.miv_view_params_update_intrinsics(), vpl);
      }
      if (au.casps.casps_miv_extension().casme_depth_quantization_params_present_flag() &&
          came.came_update_depth_quantization_flag() &&
          casme_depth_quantization_params_present_flag) {
        decodeMvpudq(came.miv_view_params_update_depth_quantization(), vpl);
      }
      if (casme_chroma_scaling_present_flag && came.came_update_chroma_scaling_flag()) {
        decodeMvpucs(came.miv_view_params_update_chroma_scaling(), vpl);
      }
    }
  }
}
} // namespace TMIV::Decoder
