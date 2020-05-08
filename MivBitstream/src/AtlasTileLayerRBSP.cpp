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

#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>

#include <TMIV/MivBitstream/MivDecoderMode.h>

#include <TMIV/Common/Common.h>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, AthType x) -> ostream & {
  switch (x) {
  case AthType::P_TILE:
    return stream << "P_TILE";
  case AthType::I_TILE:
    return stream << "I_TILE";
  case AthType::SKIP_TILE:
    return stream << "SKIP_TILE";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto operator<<(ostream &stream, FlexiblePatchOrientation x) -> ostream & {
  switch (x) {
  case FlexiblePatchOrientation::FPO_NULL:
    return stream << "FPO_NULL";
  case FlexiblePatchOrientation::FPO_SWAP:
    return stream << "FPO_SWAP";
  case FlexiblePatchOrientation::FPO_ROT90:
    return stream << "FPO_ROT90";
  case FlexiblePatchOrientation::FPO_ROT180:
    return stream << "FPO_ROT180";
  case FlexiblePatchOrientation::FPO_ROT270:
    return stream << "FPO_ROT270";
  case FlexiblePatchOrientation::FPO_MIRROR:
    return stream << "FPO_MIRROR";
  case FlexiblePatchOrientation::FPO_MROT90:
    return stream << "FPO_MROT90";
  case FlexiblePatchOrientation::FPO_MROT180:
    return stream << "FPO_MROT180";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto printTo(ostream &stream, AtduPatchMode x, AthType ath_type) -> ostream & {
  switch (ath_type) {
  case AthType::I_TILE:
    switch (x) {
    case AtduPatchMode::I_INTRA:
      return stream << "I_INTRA";
    case AtduPatchMode::I_RAW:
      return stream << "I_RAW";
    case AtduPatchMode::I_EOM:
      return stream << "I_EOM";
    case AtduPatchMode::I_END:
      return stream << "I_END";
    default:
      return stream << "[unknown:" << int(x) << "]";
    }
  case AthType::P_TILE:
    switch (x) {
    case AtduPatchMode::P_SKIP:
      return stream << "P_SKIP";
    case AtduPatchMode::P_MERGE:
      return stream << "P_MERGE";
    case AtduPatchMode::P_INTER:
      return stream << "P_INTER";
    case AtduPatchMode::P_INTRA:
      return stream << "P_INTRA";
    case AtduPatchMode::P_RAW:
      return stream << "P_RAW";
    case AtduPatchMode::P_EOM:
      return stream << "P_EOM";
    case AtduPatchMode::P_END:
      return stream << "P_END";
    default:
      return stream << "[unknown:" << int(x) << "]";
    }
  case AthType::SKIP_TILE:
    switch (x) {
    case AtduPatchMode::P_SKIP:
      return stream << "P_SKIP";
    default:
      return stream << "[unknown:" << int(x) << "]";
    }
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto AtlasTileHeader::ath_atlas_output_flag() const noexcept -> bool {
  VERIFY_V3CBITSTREAM(m_ath_atlas_output_flag.has_value());
  return *m_ath_atlas_output_flag;
}

auto AtlasTileHeader::ath_patch_size_x_info_quantizer() const noexcept -> uint8_t {
  VERIFY_V3CBITSTREAM(ath_type() != AthType::SKIP_TILE);
  return m_ath_patch_size_x_info_quantizer;
}

auto AtlasTileHeader::ath_patch_size_y_info_quantizer() const noexcept -> uint8_t {
  VERIFY_V3CBITSTREAM(ath_type() != AthType::SKIP_TILE);
  return m_ath_patch_size_y_info_quantizer;
}

auto AtlasTileHeader::ath_patch_size_x_info_quantizer(const uint8_t value) noexcept
    -> AtlasTileHeader & {
  VERIFY_V3CBITSTREAM(ath_type() != AthType::SKIP_TILE);
  m_ath_patch_size_x_info_quantizer = value;
  return *this;
}

auto AtlasTileHeader::ath_patch_size_y_info_quantizer(const uint8_t value) noexcept
    -> AtlasTileHeader & {
  VERIFY_V3CBITSTREAM(ath_type() != AthType::SKIP_TILE);
  m_ath_patch_size_y_info_quantizer = value;
  return *this;
}

auto operator<<(ostream &stream, const AtlasTileHeader &x) -> ostream & {
  stream << "ath_atlas_frame_parameter_set_id=" << int(x.ath_atlas_frame_parameter_set_id())
         << '\n';
  stream << "ath_atlas_adaptation_parameter_set_id=" << int(x.m_ath_adaptation_parameter_set_id)
         << '\n';
  stream << "ath_id=" << int(x.ath_id()) << '\n';
  stream << "ath_type=" << x.ath_type() << '\n';
  if (x.m_ath_atlas_output_flag) {
    stream << "ath_atlas_output_flag=" << boolalpha << *x.m_ath_atlas_output_flag << '\n';
  }
  stream << "ath_atlas_frm_order_cnt_lsb=" << int(x.ath_atlas_frm_order_cnt_lsb()) << '\n';
  if (x.m_ath_ref_atlas_frame_list_sps_flag) {
    stream << "ath_ref_atlas_frame_list_sps_flag=" << boolalpha
           << *x.m_ath_ref_atlas_frame_list_sps_flag << '\n';
  }
  if (x.ath_type() != AthType::SKIP_TILE) {
    if (x.m_ath_pos_min_z_quantizer) {
      stream << "ath_pos_min_z_quantizer=" << int(*x.m_ath_pos_min_z_quantizer) << '\n';
      if (x.m_ath_pos_delta_max_z_quantizer) {
        stream << "ath_pos_delta_max_z_quantizer=" << int(*x.m_ath_pos_delta_max_z_quantizer)
               << '\n';
      }
    }
    stream << "ath_patch_size_x_info_quantizer=" << int(x.ath_patch_size_x_info_quantizer())
           << '\n';
    stream << "ath_patch_size_y_info_quantizer=" << int(x.ath_patch_size_y_info_quantizer())
           << '\n';
  }
  return stream;
}

auto AtlasTileHeader::decodeFrom(InputBitstream &bitstream,
                                 const vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                 const vector<AtlasFrameParameterSetRBSP> &afpsV)
    -> AtlasTileHeader {
  auto x = AtlasTileHeader{};

  x.ath_atlas_frame_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_V3CBITSTREAM(x.ath_atlas_frame_parameter_set_id() <= 63);
  VERIFY_V3CBITSTREAM(x.ath_atlas_frame_parameter_set_id() < afpsV.size());
  const auto &afps = afpsV[x.ath_atlas_frame_parameter_set_id()];

  VERIFY_V3CBITSTREAM(afps.afps_atlas_sequence_parameter_set_id() < aspsV.size());
  const auto &asps = aspsV[afps.afps_atlas_sequence_parameter_set_id()];

  x.ath_atlas_adaptation_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());

  VERIFY_MIVBITSTREAM(afps.atlas_frame_tile_information().afti_single_tile_in_atlas_frame_flag());
  x.ath_id(0);

  x.ath_type(AthType(bitstream.getUExpGolomb<uint8_t>()));
  VERIFY_MIVBITSTREAM(x.ath_type() == AthType::I_TILE || x.ath_type() == AthType::SKIP_TILE);

  if (afps.afps_output_flag_present_flag()) {
    x.ath_atlas_output_flag(bitstream.getFlag());
  }

  x.ath_atlas_frm_order_cnt_lsb(
      bitstream.readBits<uint8_t>(asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4));

  if (asps.asps_num_ref_atlas_frame_lists_in_asps() > 0) {
    x.ath_ref_atlas_frame_list_sps_flag(bitstream.getFlag());
  }

  LIMITATION(x.ath_ref_atlas_frame_list_sps_flag());
  LIMITATION(asps.ref_list_struct(0).num_ref_entries() <= 1);

  if (x.ath_type() != AthType::SKIP_TILE) {
    if (asps.asps_normal_axis_limits_quantization_enabled_flag()) {
      x.ath_pos_min_z_quantizer(bitstream.readBits<uint8_t>(5));
      if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
        x.ath_pos_delta_max_z_quantizer(bitstream.readBits<uint8_t>(5));
      }
    }
    if (asps.asps_patch_size_quantizer_present_flag()) {
      x.ath_patch_size_x_info_quantizer(bitstream.readBits<uint8_t>(3));
      VERIFY_V3CBITSTREAM(x.ath_patch_size_x_info_quantizer() <=
                          asps.asps_log2_patch_packing_block_size());

      x.ath_patch_size_y_info_quantizer(bitstream.readBits<uint8_t>(3));
      VERIFY_V3CBITSTREAM(x.ath_patch_size_y_info_quantizer() <=
                          asps.asps_log2_patch_packing_block_size());
    } else {
      x.ath_patch_size_x_info_quantizer(asps.asps_log2_patch_packing_block_size());
      x.ath_patch_size_y_info_quantizer(asps.asps_log2_patch_packing_block_size());
    }

    VERIFY_MIVBITSTREAM(!afps.afps_raw_3d_pos_bit_count_explicit_mode_flag());
  }

  bitstream.byteAlignment();

  return x;
}

void AtlasTileHeader::encodeTo(OutputBitstream &bitstream,
                               const vector<AtlasSequenceParameterSetRBSP> &aspsV,
                               const vector<AtlasFrameParameterSetRBSP> &afpsV) const {
  VERIFY_V3CBITSTREAM(ath_atlas_frame_parameter_set_id() <= 63);
  VERIFY_V3CBITSTREAM(ath_atlas_frame_parameter_set_id() < afpsV.size());
  bitstream.putUExpGolomb(ath_atlas_frame_parameter_set_id());
  const auto &afps = afpsV[ath_atlas_frame_parameter_set_id()];

  VERIFY_V3CBITSTREAM(afps.afps_atlas_sequence_parameter_set_id() < aspsV.size());
  const auto &asps = aspsV[afps.afps_atlas_sequence_parameter_set_id()];

  bitstream.putUExpGolomb(ath_atlas_adaptation_parameter_set_id());

  VERIFY_MIVBITSTREAM(afps.atlas_frame_tile_information().afti_single_tile_in_atlas_frame_flag());
  VERIFY_V3CBITSTREAM(ath_id() == 0);

  VERIFY_MIVBITSTREAM(ath_type() == AthType::I_TILE || ath_type() == AthType::SKIP_TILE);
  bitstream.putUExpGolomb(ath_type());

  if (afps.afps_output_flag_present_flag()) {
    bitstream.putFlag(ath_atlas_output_flag());
  }

  bitstream.writeBits(ath_atlas_frm_order_cnt_lsb(),
                      asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4);

  LIMITATION(ath_ref_atlas_frame_list_sps_flag());
  LIMITATION(asps.ref_list_struct(0).num_ref_entries() <= 1);

  VERIFY_V3CBITSTREAM(asps.asps_num_ref_atlas_frame_lists_in_asps() > 0 ||
                      !ath_ref_atlas_frame_list_sps_flag());
  if (asps.asps_num_ref_atlas_frame_lists_in_asps() > 0) {
    bitstream.putFlag(ath_ref_atlas_frame_list_sps_flag());
  }

  if (ath_type() != AthType::SKIP_TILE) {
    if (asps.asps_normal_axis_limits_quantization_enabled_flag()) {
      bitstream.writeBits(ath_pos_min_z_quantizer(), 5);
      if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
        bitstream.writeBits(ath_pos_delta_max_z_quantizer(), 5);
      }
    }
    if (asps.asps_patch_size_quantizer_present_flag()) {
      VERIFY_V3CBITSTREAM(ath_patch_size_x_info_quantizer() <=
                          asps.asps_log2_patch_packing_block_size());
      bitstream.writeBits(ath_patch_size_x_info_quantizer(), 3);

      VERIFY_V3CBITSTREAM(ath_patch_size_y_info_quantizer() <=
                          asps.asps_log2_patch_packing_block_size());
      bitstream.writeBits(ath_patch_size_y_info_quantizer(), 3);
    } else {
      VERIFY_V3CBITSTREAM(ath_patch_size_x_info_quantizer() ==
                          asps.asps_log2_patch_packing_block_size());
      VERIFY_V3CBITSTREAM(ath_patch_size_y_info_quantizer() ==
                          asps.asps_log2_patch_packing_block_size());
    }

    VERIFY_MIVBITSTREAM(!afps.afps_raw_3d_pos_bit_count_explicit_mode_flag());
  }

  bitstream.byteAlignment();
}

auto operator<<(ostream &stream, const SkipPatchDataUnit & /* x */) -> ostream & { return stream; }

auto PduMivExtension::pdu_depth_occ_threshold() const noexcept -> uint32_t {
  VERIFY_MIVBITSTREAM(m_pdu_depth_occ_threshold.has_value());
  return *m_pdu_depth_occ_threshold;
}

auto PduMivExtension::printTo(ostream &stream, size_t patchIdx) const -> ostream & {
  if (m_pdu_entity_id) {
    stream << "pdu_entity_id( " << patchIdx << " )=" << pdu_entity_id() << '\n';
  }
  if (m_pdu_depth_occ_threshold) {
    stream << "pdu_depth_occ_threshold( " << patchIdx << " )=" << pdu_depth_occ_threshold() << '\n';
  }
  return stream;
}

auto PduMivExtension::decodeFrom(InputBitstream &bitstream, const V3cUnitHeader &vuh,
                                 const V3cParameterSet &vps,
                                 const AtlasSequenceParameterSetRBSP &asps) -> PduMivExtension {
  auto x = PduMivExtension{};

  if (vps.vps_miv_extension_flag()) {
    const auto &vme = vps.vps_miv_extension();
    if (vme.vme_max_entities_minus1() > 0) {
      x.pdu_entity_id(bitstream.getUVar<uint32_t>(vme.vme_max_entities_minus1() + 1));
    }
  }
  if (asps.asps_miv_extension_flag()) {
    const auto &asme = asps.asps_miv_extension();
    if (asme.asme_depth_occ_threshold_flag()) {
      const auto j = vps.atlasIdxOf(vuh.vuh_atlas_id());
      const auto &gi = vps.geometry_information(j);
      x.pdu_depth_occ_threshold(
          bitstream.readBits<uint32_t>(gi.gi_geometry_nominal_2d_bitdepth_minus1() + 1));
    }
  }
  return x;
}

void PduMivExtension::encodeTo(OutputBitstream &bitstream, const V3cUnitHeader &vuh,
                               const V3cParameterSet &vps,
                               const AtlasSequenceParameterSetRBSP &asps) const {
  if (vps.vps_miv_extension_flag() && vps.vps_miv_extension().vme_max_entities_minus1() > 0) {
    bitstream.putUVar(pdu_entity_id(), vps.vps_miv_extension().vme_max_entities_minus1() + 1);
  } else {
    VERIFY_MIVBITSTREAM(!m_pdu_entity_id.has_value());
  }
  if (asps.asps_miv_extension_flag() && asps.asps_miv_extension().asme_depth_occ_threshold_flag()) {
    const auto j = vps.atlasIdxOf(vuh.vuh_atlas_id());
    const auto &gi = vps.geometry_information(j);
    bitstream.writeBits(pdu_depth_occ_threshold(), gi.gi_geometry_nominal_2d_bitdepth_minus1() + 1);
  } else {
    VERIFY_MIVBITSTREAM(!m_pdu_depth_occ_threshold.has_value());
  }
}

auto PatchDataUnit::pdu_depth_end() const noexcept -> uint32_t {
  VERIFY_V3CBITSTREAM(m_pdu_depth_end.has_value());
  return *m_pdu_depth_end;
}

auto PatchDataUnit::pdu_miv_extension(const PduMivExtension &value) noexcept -> PatchDataUnit & {
  m_pdu_miv_extension = value;
  return *this;
}

auto PatchDataUnit::printTo(ostream &stream, size_t patchIdx) const -> ostream & {
  stream << "pdu_2d_pos_x( " << patchIdx << " )=" << pdu_2d_pos_x() << '\n';
  stream << "pdu_2d_pos_y( " << patchIdx << " )=" << pdu_2d_pos_y() << '\n';
  stream << "pdu_2d_size_x_minus1( " << patchIdx << " )=" << pdu_2d_size_x_minus1() << '\n';
  stream << "pdu_2d_size_y_minus1( " << patchIdx << " )=" << pdu_2d_size_y_minus1() << '\n';
  stream << "pdu_view_pos_x( " << patchIdx << " )=" << pdu_view_pos_x() << '\n';
  stream << "pdu_view_pos_y( " << patchIdx << " )=" << pdu_view_pos_y() << '\n';
  stream << "pdu_depth_start( " << patchIdx << " )=" << pdu_depth_start() << '\n';
  if (m_pdu_depth_end) {
    stream << "pdu_depth_end( " << patchIdx << " )=" << pdu_depth_end() << '\n';
  }
  stream << "pdu_projection_id( " << patchIdx << " )=" << pdu_projection_id() << '\n';
  stream << "pdu_orientation_index( " << patchIdx << " )=" << pdu_orientation_index() << '\n';
  if (m_pdu_miv_extension) {
    m_pdu_miv_extension->printTo(stream, patchIdx);
  }
  return stream;
}

auto PatchDataUnit::decodeFrom(InputBitstream &bitstream, const V3cUnitHeader &vuh,
                               const V3cParameterSet &vps,
                               const vector<AtlasSequenceParameterSetRBSP> &aspsV,
                               const vector<AtlasFrameParameterSetRBSP> &afpsV,
                               const AtlasTileHeader &ath) -> PatchDataUnit {
  auto x = PatchDataUnit{};

  VERIFY_V3CBITSTREAM(ath.ath_atlas_frame_parameter_set_id() < afpsV.size());
  const auto &afps = afpsV[ath.ath_atlas_frame_parameter_set_id()];

  VERIFY_V3CBITSTREAM(afps.afps_atlas_sequence_parameter_set_id() < aspsV.size());
  const auto &asps = aspsV[afps.afps_atlas_sequence_parameter_set_id()];

  x.pdu_2d_pos_x(bitstream.getUExpGolomb<uint16_t>());
  VERIFY_V3CBITSTREAM(x.pdu_2d_pos_x() < asps.asps_frame_width());

  x.pdu_2d_pos_y(bitstream.getUExpGolomb<uint16_t>());
  VERIFY_V3CBITSTREAM(x.pdu_2d_pos_y() < asps.asps_frame_height());

  x.pdu_2d_size_x_minus1(bitstream.getUExpGolomb<uint16_t>());
  x.pdu_2d_size_y_minus1(bitstream.getUExpGolomb<uint16_t>());
  x.pdu_view_pos_x(bitstream.readBits<uint16_t>(afps.afps_3d_pos_x_bit_count_minus1() + 1));
  x.pdu_view_pos_y(bitstream.readBits<uint16_t>(afps.afps_3d_pos_y_bit_count_minus1() + 1));

  VERIFY_V3CBITSTREAM(vuh.vuh_unit_type() == VuhUnitType::V3C_AD);
  const auto atlasIdx = vps.atlasIdxOf(vuh.vuh_atlas_id());
  const auto &gi = vps.geometry_information(atlasIdx);

  const auto pdu_depth_start_num_bits =
      gi.gi_geometry_3d_coordinates_bitdepth_minus1() - ath.ath_pos_min_z_quantizer() + 2;
  VERIFY_V3CBITSTREAM(pdu_depth_start_num_bits >= 0);
  x.pdu_depth_start(bitstream.readBits<uint32_t>(pdu_depth_start_num_bits));

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    const auto pdu_depth_end_num_bits =
        gi.gi_geometry_3d_coordinates_bitdepth_minus1() - ath.ath_pos_delta_max_z_quantizer() + 2;
    VERIFY_V3CBITSTREAM(pdu_depth_end_num_bits >= 0);
    x.pdu_depth_end(bitstream.readBits<uint32_t>(pdu_depth_end_num_bits));
  }

  const auto pdu_projection_id_num_bits =
      asps.asps_extended_projection_enabled_flag()
          ? ceilLog2(asps.asps_max_number_projections_minus1() + 1)
          : 3U;
  x.pdu_projection_id(bitstream.readBits<uint16_t>(pdu_projection_id_num_bits));

  const auto pdu_orientation_index_num_bits = asps.asps_use_eight_orientations_flag() ? 3 : 1;
  x.pdu_orientation_index(
      bitstream.readBits<FlexiblePatchOrientation>(pdu_orientation_index_num_bits));

  VERIFY_MIVBITSTREAM(!afps.afps_lod_mode_enabled_flag());
  VERIFY_MIVBITSTREAM(!asps.asps_point_local_reconstruction_enabled_flag());

  if (asps.asps_miv_extension_flag()) {
    x.pdu_miv_extension(PduMivExtension::decodeFrom(bitstream, vuh, vps, asps));
  }
  return x;
}

void PatchDataUnit::encodeTo(OutputBitstream &bitstream, const V3cUnitHeader &vuh,
                             const V3cParameterSet &vps,
                             const vector<AtlasSequenceParameterSetRBSP> &aspsV,
                             const vector<AtlasFrameParameterSetRBSP> &afpsV,
                             const AtlasTileHeader &ath) const {
  VERIFY_V3CBITSTREAM(ath.ath_atlas_frame_parameter_set_id() < afpsV.size());
  const auto &afps = afpsV[ath.ath_atlas_frame_parameter_set_id()];

  VERIFY_V3CBITSTREAM(afps.afps_atlas_sequence_parameter_set_id() < aspsV.size());
  const auto &asps = aspsV[afps.afps_atlas_sequence_parameter_set_id()];

  bitstream.putUExpGolomb(pdu_2d_pos_x());
  bitstream.putUExpGolomb(pdu_2d_pos_y());
  bitstream.putUExpGolomb(pdu_2d_size_x_minus1());
  bitstream.putUExpGolomb(pdu_2d_size_y_minus1());
  bitstream.writeBits(pdu_view_pos_x(), afps.afps_3d_pos_x_bit_count_minus1() + 1);
  bitstream.writeBits(pdu_view_pos_y(), afps.afps_3d_pos_y_bit_count_minus1() + 1);

  VERIFY_V3CBITSTREAM(vuh.vuh_unit_type() == VuhUnitType::V3C_AD);
  const auto atlasIdx = vps.atlasIdxOf(vuh.vuh_atlas_id());
  const auto &gi = vps.geometry_information(atlasIdx);

  const auto pdu_depth_start_num_bits =
      gi.gi_geometry_3d_coordinates_bitdepth_minus1() - ath.ath_pos_min_z_quantizer() + 2;
  VERIFY_V3CBITSTREAM(pdu_depth_start_num_bits >= 0);
  bitstream.writeBits(pdu_depth_start(), pdu_depth_start_num_bits);

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    const auto pdu_depth_end_num_bits =
        gi.gi_geometry_3d_coordinates_bitdepth_minus1() - ath.ath_pos_delta_max_z_quantizer() + 2;
    VERIFY_V3CBITSTREAM(pdu_depth_end_num_bits >= 0);
    bitstream.writeBits(pdu_depth_end(), pdu_depth_end_num_bits);
  }

  const auto pdu_projection_id_num_bits =
      asps.asps_extended_projection_enabled_flag()
          ? ceilLog2(asps.asps_max_number_projections_minus1() + 1)
          : 3U;
  VERIFY_V3CBITSTREAM((pdu_projection_id() >> pdu_projection_id_num_bits) == 0);
  bitstream.writeBits(pdu_projection_id(), pdu_projection_id_num_bits);

  if (asps.asps_use_eight_orientations_flag()) {
    bitstream.writeBits(pdu_orientation_index(), 3);
  } else {
    VERIFY_V3CBITSTREAM(pdu_orientation_index() == FlexiblePatchOrientation::FPO_NULL ||
                        pdu_orientation_index() == FlexiblePatchOrientation::FPO_SWAP);
    bitstream.writeBits(pdu_orientation_index(), 1);
  }

  VERIFY_MIVBITSTREAM(!afps.afps_lod_mode_enabled_flag());
  VERIFY_MIVBITSTREAM(!asps.asps_point_local_reconstruction_enabled_flag());

  if (asps.asps_miv_extension_flag()) {
    pdu_miv_extension().encodeTo(bitstream, vuh, vps, asps);
  } else {
    VERIFY_V3CBITSTREAM(!m_pdu_miv_extension);
  }
}

auto PatchInformationData::skip_patch_data_unit() const noexcept -> const SkipPatchDataUnit & {
  VERIFY_V3CBITSTREAM(holds_alternative<SkipPatchDataUnit>(m_data));
  return *get_if<SkipPatchDataUnit>(&m_data);
}

auto PatchInformationData::patch_data_unit() const noexcept -> const PatchDataUnit & {
  VERIFY_V3CBITSTREAM(holds_alternative<PatchDataUnit>(m_data));
  return *get_if<PatchDataUnit>(&m_data);
}

auto PatchInformationData::printTo(ostream &stream, size_t patchIdx) const -> ostream & {
  visit(overload([&](const monostate & /* unused */) { stream << "[unknown]\n"; },
                 [&](const SkipPatchDataUnit &x) { stream << x; },
                 [&](const PatchDataUnit &x) { x.printTo(stream, patchIdx); }),
        m_data);
  return stream;
}

auto PatchInformationData::operator==(const PatchInformationData &other) const noexcept -> bool {
  return data() == other.data();
}

auto PatchInformationData::operator!=(const PatchInformationData &other) const noexcept -> bool {
  return !operator==(other);
}

auto PatchInformationData::decodeFrom(InputBitstream &bitstream, const V3cUnitHeader &vuh,
                                      const V3cParameterSet &vps,
                                      const vector<AtlasSequenceParameterSetRBSP> &asps,
                                      const vector<AtlasFrameParameterSetRBSP> &afps,
                                      const AtlasTileHeader &ath, AtduPatchMode patchMode)
    -> PatchInformationData {
  if (ath.ath_type() == AthType::I_TILE) {
    VERIFY_V3CBITSTREAM(patchMode == AtduPatchMode::I_INTRA);
    return PatchInformationData{PatchDataUnit::decodeFrom(bitstream, vuh, vps, asps, afps, ath)};
  }
  if (ath.ath_type() == AthType::SKIP_TILE) {
    VERIFY_V3CBITSTREAM(patchMode == AtduPatchMode::P_SKIP);
    return PatchInformationData{SkipPatchDataUnit::decodeFrom(bitstream)};
  }
  V3CBITSTREAM_ERROR("Unknown or unsupported tile/patch mode combination");
}

void PatchInformationData::encodeTo(OutputBitstream &bitstream, const V3cUnitHeader &vuh,
                                    const V3cParameterSet &vps,
                                    const vector<AtlasSequenceParameterSetRBSP> &asps,
                                    const vector<AtlasFrameParameterSetRBSP> &afps,
                                    const AtlasTileHeader &ath, AtduPatchMode patchMode) const {
  if (ath.ath_type() == AthType::I_TILE) {
    VERIFY_V3CBITSTREAM(patchMode == AtduPatchMode::I_INTRA);
    return patch_data_unit().encodeTo(bitstream, vuh, vps, asps, afps, ath);
  }
  if (ath.ath_type() == AthType::SKIP_TILE) {
    VERIFY_V3CBITSTREAM(patchMode == AtduPatchMode::P_SKIP);
    return skip_patch_data_unit().encodeTo(bitstream);
  }
  V3CBITSTREAM_ERROR("Unknown or unsupported tile/patch mode combination");
}

auto AtlasTileDataUnit::atduTotalNumberOfPatches() const noexcept -> size_t {
  return m_vector.size();
}

auto AtlasTileDataUnit::atdu_patch_mode(size_t p) const -> AtduPatchMode {
  VERIFY_V3CBITSTREAM(p < m_vector.size());
  return m_vector[p].first;
}

auto AtlasTileDataUnit::patch_information_data(size_t p) const
    -> const PatchInformationData & {
  VERIFY_V3CBITSTREAM(p < m_vector.size());
  return m_vector[p].second;
}

auto AtlasTileDataUnit::printTo(ostream &stream, AthType ath_type) const -> ostream & {
  visit([&](const auto p, const AtduPatchMode patch_mode,
            const PatchInformationData &patch_information_data) {
    stream << "atdu_patch_mode[ " << p << " ]=";
    MivBitstream::printTo(stream, patch_mode, ath_type) << '\n';
    patch_information_data.printTo(stream, p);
  });
  return stream;
}

auto AtlasTileDataUnit::operator==(const AtlasTileDataUnit &other) const -> bool {
  return m_vector == other.m_vector;
}

auto AtlasTileDataUnit::operator!=(const AtlasTileDataUnit &other) const -> bool {
  return !operator==(other);
}

auto AtlasTileDataUnit::decodeFrom(InputBitstream &bitstream, const V3cUnitHeader &vuh,
                                        const V3cParameterSet &vps,
                                        const vector<AtlasSequenceParameterSetRBSP> &asps,
                                        const vector<AtlasFrameParameterSetRBSP> &afps,
                                        const AtlasTileHeader &ath) -> AtlasTileDataUnit {
  VERIFY_V3CBITSTREAM(ath.ath_type() == AthType::I_TILE || ath.ath_type() == AthType::P_TILE);
  VERIFY_MIVBITSTREAM(ath.ath_type() == AthType::I_TILE);

  auto x = AtlasTileDataUnit::Vector{};
  auto patch_mode = bitstream.getUExpGolomb<AtduPatchMode>();

  while (patch_mode != AtduPatchMode::I_END) {
    x.emplace_back(patch_mode, PatchInformationData::decodeFrom(bitstream, vuh, vps, asps, afps,
                                                                ath, patch_mode));
    VERIFY_MIVBITSTREAM(patch_mode == AtduPatchMode::I_INTRA);
    patch_mode = bitstream.getUExpGolomb<AtduPatchMode>();
  }

  bitstream.byteAlignment();
  return AtlasTileDataUnit{x};
}

void AtlasTileDataUnit::encodeTo(OutputBitstream &bitstream, const V3cUnitHeader &vuh,
                                      const V3cParameterSet &vps,
                                      const vector<AtlasSequenceParameterSetRBSP> &asps,
                                      const vector<AtlasFrameParameterSetRBSP> &afps,
                                      const AtlasTileHeader &ath) const {
  VERIFY_V3CBITSTREAM(ath.ath_type() == AthType::I_TILE || ath.ath_type() == AthType::P_TILE);
  VERIFY_MIVBITSTREAM(ath.ath_type() == AthType::I_TILE);

  visit([&](const auto /* p */, const AtduPatchMode patch_mode,
            const PatchInformationData &patch_information_data) {
    bitstream.putUExpGolomb(patch_mode);
    patch_information_data.encodeTo(bitstream, vuh, vps, asps, afps, ath, patch_mode);
  });

  bitstream.putUExpGolomb(AtduPatchMode::I_END);
  bitstream.byteAlignment();
}

auto AtlasTileLayerRBSP::atlas_tile_data_unit() const noexcept
    -> const AtlasTileDataUnit & {
  VERIFY_V3CBITSTREAM(m_atlas_tile_data_unit);
  return *m_atlas_tile_data_unit;
}

auto operator<<(ostream &stream, const AtlasTileLayerRBSP &x) -> ostream & {
  stream << x.atlas_tile_header();
  if (x.atlas_tile_header().ath_type() != AthType::SKIP_TILE) {
    x.atlas_tile_data_unit().printTo(stream, x.atlas_tile_header().ath_type());
  }
  return stream;
}

auto AtlasTileLayerRBSP::operator==(const AtlasTileLayerRBSP &other) const noexcept -> bool {
  return atlas_tile_header() == other.atlas_tile_header() &&
         m_atlas_tile_data_unit == other.m_atlas_tile_data_unit;
}

auto AtlasTileLayerRBSP::operator!=(const AtlasTileLayerRBSP &other) const noexcept -> bool {
  return !operator==(other);
}

auto AtlasTileLayerRBSP::decodeFrom(istream &stream, const V3cUnitHeader &vuh,
                                    const V3cParameterSet &vps,
                                    const vector<AtlasSequenceParameterSetRBSP> &asps,
                                    const vector<AtlasFrameParameterSetRBSP> &afps)
    -> AtlasTileLayerRBSP {
  InputBitstream bitstream{stream};

  const auto ath = AtlasTileHeader::decodeFrom(bitstream, asps, afps);
  auto atl = AtlasTileLayerRBSP{ath};

  if (ath.ath_type() != AthType::SKIP_TILE) {
    atl = {ath, AtlasTileDataUnit::decodeFrom(bitstream, vuh, vps, asps, afps, ath)};
  }

  bitstream.rbspTrailingBits();
  return atl;
}

void AtlasTileLayerRBSP::encodeTo(ostream &stream, const V3cUnitHeader &vuh,
                                  const V3cParameterSet &vps,
                                  const vector<AtlasSequenceParameterSetRBSP> &asps,
                                  const vector<AtlasFrameParameterSetRBSP> &afps) const {
  OutputBitstream bitstream{stream};

  const auto &ath = atlas_tile_header();
  ath.encodeTo(bitstream, asps, afps);

  if (ath.ath_type() != AthType::SKIP_TILE) {
    atlas_tile_data_unit().encodeTo(bitstream, vuh, vps, asps, afps, ath);
  }

  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream
