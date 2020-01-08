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

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>
#include <TMIV/Metadata/IvAccessUnitParams.h>
#include <TMIV/Metadata/IvSequenceParams.h>
#include <TMIV/MivBitstream/MivParameterSet.h>
#include <TMIV/MivBitstream/MivPatchUnit.h>
#include <TMIV/VpccBitstream/VpccSampleStreamFormat.h>

#include <fstream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::MivBitstream;
using namespace TMIV::VpccBitstream;

// TODO: Where to define this
enum : uint8_t {
  CODEC_HEVC = 0,                 // From TMC2
  CODEC_OCCUPANCY_IN_GEOMETRY = 1 // For MIV
};

TEST_CASE("Transcode from TMIV 3.0 to proposal", "[MIV Bitstream]") {
  ifstream is{"C:/Data/ATL_SA_R0_Tm_c00.bit", ios::binary};
  InputBitstream ibs{is};
  const auto ivs = IvSequenceParams::decodeFrom(ibs);
  auto ivau = array<IvAccessUnitParams, 4>{};
  for (int i = 0; i < 4; ++i) {
    ivau[i] = IvAccessUnitParams::decodeFrom(ibs, ivs);
  }

  const auto &atlasSizes = ivau.front().atlasParamsList->atlasSizes;
  const auto atlasCount = uint8_t(atlasSizes.size());

  ofstream os{"C:/Data/ATL_SA_R0_Tm_c00.miv", ios::binary};

  const auto ssvh = SampleStreamVpccHeader{4};
  ssvh.encodeTo(os);
  cout << "\nsample_stream_vpcc_header():\n" << ssvh;

  auto vps = VpccParameterSet{};
  vps.profile_tier_level()
      .ptl_tier_flag(false)
      .ptl_profile_codec_group_idc(PtlProfileCodecGroupIdc::HEVC_Main10)
      .ptl_profile_pcc_toolset_idc(PtlProfilePccToolsetIdc::Basic)
      .ptl_profile_reconstruction_idc(PtlProfileReconstructionIdc::Unconstrained)
      .ptl_level_idc(PtlLevelIdc::Level_1_0);
  vps.vps_vpcc_parameter_set_id(0)
      .vps_atlas_count(uint8_t(atlasSizes.size()))
      .vps_extension_present_flag(true);

  for (uint8_t atlasId = 0; atlasId < atlasCount; ++atlasId) {
    vps.vps_frame_width(atlasId, atlasSizes[atlasId].x())
        .vps_frame_height(atlasId, atlasSizes[atlasId].y())
        .vps_map_count(atlasId, 1)
        .vps_raw_patch_enabled_flag(atlasId, false);

    // Occupancy is part of depth map
    vps.occupancy_information(atlasId).oi_occupancy_codec_id(CODEC_OCCUPANCY_IN_GEOMETRY);
    vps.occupancy_information(atlasId).oi_occupancy_nominal_2d_bitdepth(1);
    vps.occupancy_information(atlasId).oi_lossy_occupancy_map_compression_threshold(0);
    vps.occupancy_information(atlasId).oi_occupancy_MSB_align_flag(false);

    // Geometry is HEVC Main 10 without bit depth alignment nor 3D coordinates
    vps.geometry_information(atlasId).gi_geometry_codec_id(CODEC_HEVC);
    vps.geometry_information(atlasId).gi_geometry_nominal_2d_bitdepth(10);
    vps.geometry_information(atlasId).gi_geometry_MSB_align_flag(false);
    vps.geometry_information(atlasId).gi_geometry_3d_coordinates_bitdepth(1);

    // Single attribute is HEVC Main 10 YUV texture
    vps.attribute_information(atlasId).ai_attribute_count(1);
    vps.attribute_information(atlasId).ai_attribute_type_id(0, AiAttributeTypeId::ATTR_TEXTURE);
    vps.attribute_information(atlasId).ai_attribute_codec_id(0, CODEC_HEVC);
    vps.attribute_information(atlasId).ai_attribute_dimension(0, 3);
    vps.attribute_information(atlasId).ai_attribute_nominal_2d_bitdepth(0, 10);
    vps.attribute_information(atlasId).ai_attribute_MSB_align_flag(false);
  }

  auto msp = MivSequenceParams{};
  msp.view_params_list(ivs.viewParamsList);
  if (ivs.viewingSpace) {
    msp.viewing_space(*ivs.viewingSpace);
  }

  auto mps = MivParameterSet{vps, msp};
  mps.updateOverridePduProjectionIdNumBits();
  ostringstream osubstream;
  cout << "\nmiv_parameter_set():\n" << mps;
  mps.encodeTo(osubstream);

  const auto ssu = SampleStreamVpccUnit{osubstream.str()};
  cout << "\nsample_stream_vpcc_unit():\n" << ssu;
  ssu.encodeTo(os, ssvh);
}
