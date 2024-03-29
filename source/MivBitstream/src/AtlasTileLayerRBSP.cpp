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

#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/Formatters.h>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, AthType x) -> std::ostream & {
  switch (x) {
  case AthType::P_TILE:
    return stream << "P_TILE";
  case AthType::I_TILE:
    return stream << "I_TILE";
  case AthType::SKIP_TILE:
    return stream << "SKIP_TILE";
  default:
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
  }
}

auto operator<<(std::ostream &stream, FlexiblePatchOrientation x) -> std::ostream & {
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
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
  }
}

auto printTo(std::ostream &stream, AtduPatchMode x, AthType ath_type) -> std::ostream & {
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
      return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
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
      return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
    }
  case AthType::SKIP_TILE:
    switch (x) {
    case AtduPatchMode::P_SKIP:
      return stream << "P_SKIP";
    default:
      return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
    }
  default:
    return stream << "[unknown:" << static_cast<int32_t>(x) << "]";
  }
}

auto AtlasTileHeader::ath_atlas_output_flag() const -> bool {
  VERIFY_V3CBITSTREAM(m_ath_atlas_output_flag.has_value());
  return *m_ath_atlas_output_flag;
}

auto AtlasTileHeader::ath_patch_size_x_info_quantizer() const -> uint8_t {
  VERIFY_V3CBITSTREAM(ath_type() != AthType::SKIP_TILE);
  VERIFY_V3CBITSTREAM(m_ath_patch_size_x_info_quantizer.has_value());
  return *m_ath_patch_size_x_info_quantizer;
}

auto AtlasTileHeader::ath_patch_size_y_info_quantizer() const -> uint8_t {
  VERIFY_V3CBITSTREAM(ath_type() != AthType::SKIP_TILE);
  VERIFY_V3CBITSTREAM(m_ath_patch_size_x_info_quantizer.has_value());
  return *m_ath_patch_size_y_info_quantizer;
}

auto AtlasTileHeader::ath_patch_size_x_info_quantizer(const uint8_t value) noexcept
    -> AtlasTileHeader & {
  PRECONDITION(ath_type() != AthType::SKIP_TILE);
  m_ath_patch_size_x_info_quantizer = value;
  return *this;
}

auto AtlasTileHeader::ath_patch_size_y_info_quantizer(const uint8_t value) noexcept
    -> AtlasTileHeader & {
  PRECONDITION(ath_type() != AthType::SKIP_TILE);
  m_ath_patch_size_y_info_quantizer = value;
  return *this;
}

auto operator<<(std::ostream &stream, const AtlasTileHeader &x) -> std::ostream & {
  if (x.m_ath_no_output_of_prior_atlas_frames_flag.has_value()) {
    stream << "ath_no_output_of_prior_atlas_frames_flag=" << std::boolalpha
           << x.ath_no_output_of_prior_atlas_frames_flag() << '\n';
  }
  stream << "ath_atlas_frame_parameter_set_id=" << int32_t{x.ath_atlas_frame_parameter_set_id()}
         << '\n';
  stream << "ath_atlas_adaptation_parameter_set_id=" << int32_t{x.m_ath_adaptation_parameter_set_id}
         << '\n';
  stream << "ath_id=" << int32_t{x.ath_id()} << '\n';
  stream << "ath_type=" << x.ath_type() << '\n';
  if (x.m_ath_atlas_output_flag) {
    stream << "ath_atlas_output_flag=" << std::boolalpha << *x.m_ath_atlas_output_flag << '\n';
  }
  stream << "ath_atlas_frm_order_cnt_lsb=" << int32_t{x.ath_atlas_frm_order_cnt_lsb()} << '\n';
  if (x.m_ath_ref_atlas_frame_list_asps_flag) {
    stream << "ath_ref_atlas_frame_list_asps_flag=" << std::boolalpha
           << *x.m_ath_ref_atlas_frame_list_asps_flag << '\n';
  }
  if (x.ath_type() != AthType::SKIP_TILE) {
    if (x.m_ath_pos_min_d_quantizer) {
      stream << "ath_pos_min_d_quantizer=" << int32_t{*x.m_ath_pos_min_d_quantizer} << '\n';
      if (x.m_ath_pos_delta_max_d_quantizer) {
        stream << "ath_pos_delta_max_d_quantizer=" << int32_t{*x.m_ath_pos_delta_max_d_quantizer}
               << '\n';
      }
    }
    if (x.m_ath_patch_size_x_info_quantizer || x.m_ath_patch_size_y_info_quantizer) {
      stream << "ath_patch_size_x_info_quantizer=" << int32_t{x.ath_patch_size_x_info_quantizer()}
             << '\n';
      stream << "ath_patch_size_y_info_quantizer=" << int32_t{x.ath_patch_size_y_info_quantizer()}
             << '\n';
    }
  }
  return stream;
}

auto AtlasTileHeader::decodeFrom(Common::InputBitstream &bitstream, const NalUnitHeader &nuh,
                                 const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                 const std::vector<AtlasFrameParameterSetRBSP> &afpsV)
    -> AtlasTileHeader {
  auto x = AtlasTileHeader{};

  if (NalUnitType::NAL_BLA_W_LP <= nuh.nal_unit_type() &&
      nuh.nal_unit_type() <= NalUnitType::NAL_RSV_IRAP_ACL_29) {
    x.ath_no_output_of_prior_atlas_frames_flag(bitstream.getFlag());
  }

  x.ath_atlas_frame_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_V3CBITSTREAM(x.ath_atlas_frame_parameter_set_id() <= 63);
  const auto &afps = afpsById(afpsV, x.ath_atlas_frame_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps.afps_atlas_sequence_parameter_set_id());

  x.ath_atlas_adaptation_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());

  const auto &afti = afps.atlas_frame_tile_information();

  if (afti.afti_single_tile_in_atlas_frame_flag()) {
    x.ath_id(0);
  } else {
    x.ath_id(bitstream.readBits<uint8_t>(afti.aftiSignalledTileIDBitCount()));
  }

  x.ath_type(bitstream.getUExpGolomb<AthType>());

  if (afps.afps_output_flag_present_flag()) {
    x.ath_atlas_output_flag(bitstream.getFlag());
  }

  x.ath_atlas_frm_order_cnt_lsb(
      bitstream.readBits<uint16_t>(asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4));

  if (asps.asps_num_ref_atlas_frame_lists_in_asps() > 0) {
    x.ath_ref_atlas_frame_list_asps_flag(bitstream.getFlag());
  }

  LIMITATION(x.ath_ref_atlas_frame_list_asps_flag());
  LIMITATION(asps.ref_list_struct(0).num_ref_entries() <= 1);

  if (x.ath_type() != AthType::SKIP_TILE) {
    if (asps.asps_normal_axis_limits_quantization_enabled_flag()) {
      x.ath_pos_min_d_quantizer(bitstream.readBits<uint8_t>(5));
      if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
        x.ath_pos_delta_max_d_quantizer(bitstream.readBits<uint8_t>(5));
      }
    }
    if (asps.asps_patch_size_quantizer_present_flag()) {
      x.ath_patch_size_x_info_quantizer(bitstream.readBits<uint8_t>(3));
      VERIFY_V3CBITSTREAM(x.ath_patch_size_x_info_quantizer() <=
                          asps.asps_log2_patch_packing_block_size());

      x.ath_patch_size_y_info_quantizer(bitstream.readBits<uint8_t>(3));
      VERIFY_V3CBITSTREAM(x.ath_patch_size_y_info_quantizer() <=
                          asps.asps_log2_patch_packing_block_size());
    }

    VERIFY_MIVBITSTREAM(!afps.afps_raw_3d_offset_bit_count_explicit_mode_flag());
  }

  bitstream.byteAlignment();

  return x;
}

void AtlasTileHeader::encodeTo(Common::OutputBitstream &bitstream, const NalUnitHeader &nuh,
                               const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                               const std::vector<AtlasFrameParameterSetRBSP> &afpsV) const {
  if (NalUnitType::NAL_BLA_W_LP <= nuh.nal_unit_type() &&
      nuh.nal_unit_type() <= NalUnitType::NAL_RSV_IRAP_ACL_29) {
    bitstream.putFlag(ath_no_output_of_prior_atlas_frames_flag());
  }

  PRECONDITION(ath_atlas_frame_parameter_set_id() <= 63);
  bitstream.putUExpGolomb(ath_atlas_frame_parameter_set_id());

  const auto &afps = afpsById(afpsV, ath_atlas_frame_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps.afps_atlas_sequence_parameter_set_id());

  bitstream.putUExpGolomb(ath_atlas_adaptation_parameter_set_id());

  const auto &afti = afps.atlas_frame_tile_information();

  if (afti.afti_single_tile_in_atlas_frame_flag()) {
    PRECONDITION(ath_id() == 0);
  } else {
    bitstream.writeBits(ath_id(), afti.aftiSignalledTileIDBitCount());
  }
  bitstream.putUExpGolomb(ath_type());

  if (afps.afps_output_flag_present_flag()) {
    bitstream.putFlag(ath_atlas_output_flag());
  }

  bitstream.writeBits(ath_atlas_frm_order_cnt_lsb(),
                      asps.asps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4);

  LIMITATION(ath_ref_atlas_frame_list_asps_flag());
  LIMITATION(asps.ref_list_struct(0).num_ref_entries() <= 1);

  PRECONDITION(asps.asps_num_ref_atlas_frame_lists_in_asps() > 0 ||
               !ath_ref_atlas_frame_list_asps_flag());
  if (asps.asps_num_ref_atlas_frame_lists_in_asps() > 0) {
    bitstream.putFlag(ath_ref_atlas_frame_list_asps_flag());
  }

  if (ath_type() != AthType::SKIP_TILE) {
    if (asps.asps_normal_axis_limits_quantization_enabled_flag()) {
      bitstream.writeBits(ath_pos_min_d_quantizer(), 5);
      if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
        bitstream.writeBits(ath_pos_delta_max_d_quantizer(), 5);
      }
    }
    if (asps.asps_patch_size_quantizer_present_flag()) {
      PRECONDITION(ath_patch_size_x_info_quantizer() <= asps.asps_log2_patch_packing_block_size());
      bitstream.writeBits(ath_patch_size_x_info_quantizer(), 3);

      PRECONDITION(ath_patch_size_y_info_quantizer() <= asps.asps_log2_patch_packing_block_size());
      bitstream.writeBits(ath_patch_size_y_info_quantizer(), 3);
    }

    PRECONDITION(!afps.afps_raw_3d_offset_bit_count_explicit_mode_flag());
  }

  bitstream.byteAlignment();
}

auto operator<<(std::ostream &stream, const SkipPatchDataUnit & /* x */) -> std::ostream & {
  return stream;
}

auto PduMivExtension::pdu_depth_occ_threshold() const -> Common::SampleValue {
  VERIFY_MIVBITSTREAM(m_pdu_depth_occ_threshold.has_value());
  return *m_pdu_depth_occ_threshold;
}

auto PduMivExtension::pdu_texture_offset(uint8_t c) const -> Common::SampleValue {
  return m_pdu_texture_offset.value_or(Common::Vec3sv{})[c];
}

auto PduMivExtension::printTo(std::ostream &stream, uint32_t tileId, size_t patchIdx) const
    -> std::ostream & {
  if (m_pdu_entity_id) {
    TMIV_FMT::print(stream, "pdu_entity_id[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_entity_id());
  }
  if (m_pdu_depth_occ_threshold) {
    TMIV_FMT::print(stream, "pdu_depth_occ_threshold[ {} ][ {} ]={}\n", tileId, patchIdx,
                    pdu_depth_occ_threshold());
  }
  if (m_pdu_texture_offset) {
    for (uint8_t c = 0; c < 3; ++c) {
      TMIV_FMT::print(stream, "pdu_texture_offset[ {} ][ {} ][ {} ]={}\n", tileId, patchIdx, c,
                      pdu_texture_offset(c));
    }
  }
  if (m_pdu_inpaint_flag) {
    TMIV_FMT::print(stream, "pdu_inpaint_flag[ {} ][ {} ]={}\n", tileId, patchIdx,
                    pdu_inpaint_flag());
  }
  if (m_pdu_2d_margin_u) {
    TMIV_FMT::print(stream, "pdu_2d_margin_u[ {} ][ {} ]={}\n", tileId, patchIdx,
                    pdu_2d_margin_u());
  }
  if (m_pdu_2d_margin_v) {
    TMIV_FMT::print(stream, "pdu_2d_margin_v[ {} ][ {} ]={}\n", tileId, patchIdx,
                    pdu_2d_margin_v());
  }
  return stream;
}

auto PduMivExtension::operator==(const PduMivExtension &other) const -> bool {
  return pdu_entity_id() == other.pdu_entity_id() &&
         m_pdu_depth_occ_threshold == other.m_pdu_depth_occ_threshold &&
         m_pdu_texture_offset == other.m_pdu_texture_offset &&
         pdu_inpaint_flag() == other.pdu_inpaint_flag() &&
         m_pdu_2d_margin_u == other.m_pdu_2d_margin_u &&
         m_pdu_2d_margin_v == other.m_pdu_2d_margin_v;
}

auto PduMivExtension::operator!=(const PduMivExtension &other) const -> bool {
  return !operator==(other);
}

auto PduMivExtension::decodeFrom(Common::InputBitstream &bitstream,
                                 const AtlasSequenceParameterSetRBSP &asps) -> PduMivExtension {
  auto x = PduMivExtension{};

  const auto &asme = asps.asps_miv_extension();
  if (0 < asme.asme_max_entity_id()) {
    x.pdu_entity_id(bitstream.getUVar<Common::SampleValue>(asme.asme_max_entity_id() + 1));
  }
  if (asme.asme_depth_occ_threshold_flag()) {
    x.pdu_depth_occ_threshold(
        bitstream.readBits<Common::SampleValue>(asps.asps_geometry_2d_bit_depth_minus1() + 1));
  }
  if (asme.asme_patch_texture_offset_enabled_flag()) {
    int32_t bits = asps.asps_miv_extension().asme_patch_texture_offset_bit_depth_minus1() + 1;

    for (uint8_t c = 0; c < 3; ++c) {
      x.pdu_texture_offset(c, bitstream.readBits<uint16_t>(bits));
    }
  }
  if (asme.asme_inpaint_enabled_flag()) {
    x.pdu_inpaint_flag(bitstream.getFlag());
  }
  if (asps.asps_miv_2_extension_present_flag() &&
      asps.asps_miv_2_extension().asme_patch_margin_enabled_flag()) {
    const auto bits = asps.asps_log2_patch_packing_block_size() - 1;
    x.pdu_2d_margin_u(bitstream.readBits<uint16_t>(bits));
    x.pdu_2d_margin_v(bitstream.readBits<uint16_t>(bits));
  }
  return x;
}

void PduMivExtension::encodeTo(Common::OutputBitstream &bitstream,
                               const AtlasSequenceParameterSetRBSP &asps) const {
  const auto &asme = asps.asps_miv_extension();
  if (0 < asme.asme_max_entity_id()) {
    bitstream.putUVar(pdu_entity_id(), asme.asme_max_entity_id() + 1);
  } else {
    PRECONDITION(pdu_entity_id() == 0);
  }
  if (asme.asme_depth_occ_threshold_flag()) {
    bitstream.writeBits(pdu_depth_occ_threshold(), asps.asps_geometry_2d_bit_depth_minus1() + 1);
  } else {
    PRECONDITION(!m_pdu_depth_occ_threshold.has_value());
  }
  if (asme.asme_patch_texture_offset_enabled_flag()) {
    const auto bits = asps.asps_miv_extension().asme_patch_texture_offset_bit_depth_minus1() + 1;

    for (uint8_t c = 0; c < 3; ++c) {
      bitstream.writeBits(pdu_texture_offset(c), bits);
    }
  }
  if (asme.asme_inpaint_enabled_flag()) {
    PRECONDITION(m_pdu_inpaint_flag.has_value());
    bitstream.putFlag(pdu_inpaint_flag());
  }
  if (asps.asps_miv_2_extension_present_flag() &&
      asps.asps_miv_2_extension().asme_patch_margin_enabled_flag()) {
    const auto bits = asps.asps_log2_patch_packing_block_size() - 1;
    bitstream.writeBits(pdu_2d_margin_u(), bits);
    bitstream.writeBits(pdu_2d_margin_v(), bits);
  }
}

auto PatchDataUnit::pdu_3d_range_d() const -> Common::SampleValue {
  VERIFY_V3CBITSTREAM(m_pdu_3d_range_d.has_value());
  return *m_pdu_3d_range_d;
}

auto PatchDataUnit::pdu_miv_extension(const PduMivExtension &value) noexcept -> PatchDataUnit & {
  m_pdu_miv_extension = value;
  return *this;
}

auto PatchDataUnit::printTo(std::ostream &stream, uint32_t tileId, size_t patchIdx) const
    -> std::ostream & {
  TMIV_FMT::print(stream, "pdu_2d_pos_x[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_2d_pos_x());
  TMIV_FMT::print(stream, "pdu_2d_pos_y[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_2d_pos_y());
  TMIV_FMT::print(stream, "pdu_2d_size_x_minus1[ {} ][ {} ]={}\n", tileId, patchIdx,
                  pdu_2d_size_x_minus1());
  TMIV_FMT::print(stream, "pdu_2d_size_y_minus1[ {} ][ {} ]={}\n", tileId, patchIdx,
                  pdu_2d_size_y_minus1());
  TMIV_FMT::print(stream, "pdu_3d_offset_u[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_3d_offset_u());
  TMIV_FMT::print(stream, "pdu_3d_offset_v[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_3d_offset_v());
  TMIV_FMT::print(stream, "pdu_3d_offset_d[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_3d_offset_d());

  if (m_pdu_3d_range_d) {
    TMIV_FMT::print(stream, "pdu_3d_range_d[ {} ][ {} ]={}\n", tileId, patchIdx, pdu_3d_range_d());
  }

  TMIV_FMT::print(stream, "pdu_projection_id[ {} ][ {} ]={}\n", tileId, patchIdx,
                  pdu_projection_id());
  TMIV_FMT::print(stream, "pdu_orientation_index[ {} ][ {} ]={}\n", tileId, patchIdx,
                  pdu_orientation_index());

  if (m_pdu_lod_enabled_flag) {
    TMIV_FMT::print(stream, "pdu_lod_enabled_flag[ {} ][ {} ]={}\n", tileId, patchIdx,
                    pdu_lod_enabled_flag());

    if (pdu_lod_enabled_flag()) {
      TMIV_FMT::print(stream, "pdu_lod_scale_x_minus1[ {} ][ {} ]={}\n", tileId, patchIdx,
                      pdu_lod_scale_x_minus1());
      TMIV_FMT::print(stream, "pdu_lod_scale_y_idc[ {} ][ {} ]={}\n", tileId, patchIdx,
                      pdu_lod_scale_y_idc());
    }
  }
  if (m_pdu_miv_extension) {
    m_pdu_miv_extension->printTo(stream, tileId, patchIdx);
  }
  return stream;
}

auto PatchDataUnit::decodeFrom(Common::InputBitstream &bitstream,
                               const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                               const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                               const AtlasTileHeader &ath) -> PatchDataUnit {
  auto x = PatchDataUnit{};

  const auto &afps = afpsById(afpsV, ath.ath_atlas_frame_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps.afps_atlas_sequence_parameter_set_id());

  const auto pdu3dOffsetUVNumBits = asps.asps_geometry_3d_bit_depth_minus1() + 1U;
  const auto pdu3dOffsetDNumBits =
      asps.asps_geometry_3d_bit_depth_minus1() - ath.ath_pos_min_d_quantizer() + 1;
  const auto rangeDBitDepth = std::min(asps.asps_geometry_2d_bit_depth_minus1() + 1,
                                       asps.asps_geometry_3d_bit_depth_minus1() + 1);
  const auto pdu3dRangeDNumBits = rangeDBitDepth - ath.ath_pos_delta_max_d_quantizer();
  const auto pduProjectionIdNumBits =
      Common::ceilLog2(asps.asps_max_number_projections_minus1() + 1ULL);
  const auto pduOrientationIndexNumBits = asps.asps_use_eight_orientations_flag() ? 3 : 1;

  x.pdu_2d_pos_x(bitstream.getUExpGolomb<uint32_t>());
  x.pdu_2d_pos_y(bitstream.getUExpGolomb<uint32_t>());
  x.pdu_2d_size_x_minus1(bitstream.getUExpGolomb<uint32_t>());
  x.pdu_2d_size_y_minus1(bitstream.getUExpGolomb<uint32_t>());
  x.pdu_3d_offset_u(bitstream.readBits<uint32_t>(pdu3dOffsetUVNumBits));
  x.pdu_3d_offset_v(bitstream.readBits<uint32_t>(pdu3dOffsetUVNumBits));

  VERIFY_V3CBITSTREAM(pdu3dOffsetDNumBits >= 0);
  x.pdu_3d_offset_d(bitstream.readBits<uint32_t>(pdu3dOffsetDNumBits));

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    VERIFY_V3CBITSTREAM(pdu3dRangeDNumBits >= 0);
    x.pdu_3d_range_d(bitstream.readBits<uint32_t>(pdu3dRangeDNumBits));
  }

  x.pdu_projection_id(ViewId::decodeFrom(bitstream, pduProjectionIdNumBits));
  x.pdu_orientation_index(bitstream.readBits<FlexiblePatchOrientation>(pduOrientationIndexNumBits));

  if (afps.afps_lod_mode_enabled_flag()) {
    x.pdu_lod_enabled_flag(bitstream.getFlag());
    if (x.pdu_lod_enabled_flag()) {
      x.pdu_lod_scale_x_minus1(bitstream.getUExpGolomb<uint32_t>());
      x.pdu_lod_scale_y_idc(bitstream.getUExpGolomb<uint32_t>());
    }
  }

  VERIFY_MIVBITSTREAM(!asps.asps_plr_enabled_flag());

  if (asps.asps_miv_extension_present_flag()) {
    x.pdu_miv_extension(PduMivExtension::decodeFrom(bitstream, asps));
  }
  return x;
}

void PatchDataUnit::encodeTo(Common::OutputBitstream &bitstream,
                             const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                             const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                             const AtlasTileHeader &ath) const {
  const auto &afps = afpsById(afpsV, ath.ath_atlas_frame_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps.afps_atlas_sequence_parameter_set_id());

  const auto pdu3dOffsetUVNumBits = asps.asps_geometry_3d_bit_depth_minus1() + 1U;
  const auto pdu3dOffsetDNumBits =
      asps.asps_geometry_3d_bit_depth_minus1() - ath.ath_pos_min_d_quantizer() + 1;
  const auto rangeDBitDepth = std::min(asps.asps_geometry_2d_bit_depth_minus1() + 1,
                                       asps.asps_geometry_3d_bit_depth_minus1() + 1);
  const auto pdu3dRangeDNumBits = rangeDBitDepth - ath.ath_pos_delta_max_d_quantizer();
  const auto pduProjectionIdNumBits =
      Common::ceilLog2(asps.asps_max_number_projections_minus1() + 1ULL);
  const auto pduOrientationIndexNumBits = asps.asps_use_eight_orientations_flag() ? 3 : 1;

  bitstream.putUExpGolomb(pdu_2d_pos_x());
  bitstream.putUExpGolomb(pdu_2d_pos_y());
  bitstream.putUExpGolomb(pdu_2d_size_x_minus1());
  bitstream.putUExpGolomb(pdu_2d_size_y_minus1());
  bitstream.writeBits(pdu_3d_offset_u(), pdu3dOffsetUVNumBits);
  bitstream.writeBits(pdu_3d_offset_v(), pdu3dOffsetUVNumBits);

  PRECONDITION(pdu3dOffsetDNumBits >= 0);
  bitstream.writeBits(pdu_3d_offset_d(), pdu3dOffsetDNumBits);

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    PRECONDITION(pdu3dRangeDNumBits >= 0);
    bitstream.writeBits(pdu_3d_range_d(), pdu3dRangeDNumBits);
  }

  pdu_projection_id().encodeTo(bitstream, pduProjectionIdNumBits);
  bitstream.writeBits(pdu_orientation_index(), pduOrientationIndexNumBits);

  if (afps.afps_lod_mode_enabled_flag()) {
    bitstream.putFlag(pdu_lod_enabled_flag());
    if (pdu_lod_enabled_flag()) {
      bitstream.putUExpGolomb(pdu_lod_scale_x_minus1());
      bitstream.putUExpGolomb(pdu_lod_scale_y_idc());
    }
  }

  PRECONDITION(!asps.asps_plr_enabled_flag());

  if (asps.asps_miv_extension_present_flag()) {
    pdu_miv_extension().encodeTo(bitstream, asps);
  } else {
    PRECONDITION(!m_pdu_miv_extension);
  }
}

auto InterPatchDataUnit::ipdu_ref_index() const -> int32_t { return m_ipdu_ref_index.value_or(0); }

auto InterPatchDataUnit::ipdu_patch_index() const -> int32_t { return m_ipdu_patch_index; }

auto InterPatchDataUnit::ipdu_2d_pos_x() const -> int32_t { return m_ipdu_2d_pos_x; }

auto InterPatchDataUnit::ipdu_2d_pos_y() const -> int32_t { return m_ipdu_2d_pos_y; }

auto InterPatchDataUnit::ipdu_2d_delta_size_x() const -> int32_t { return m_ipdu_2d_delta_size_x; }

auto InterPatchDataUnit::ipdu_2d_delta_size_y() const -> int32_t { return m_ipdu_2d_delta_size_y; }

auto InterPatchDataUnit::ipdu_3d_offset_u() const -> int32_t { return m_ipdu_3d_offset_u; }

auto InterPatchDataUnit::ipdu_3d_offset_v() const -> int32_t { return m_ipdu_3d_offset_v; }

auto InterPatchDataUnit::ipdu_3d_offset_d() const -> int32_t { return m_ipdu_3d_offset_d; }

auto InterPatchDataUnit::ipdu_3d_range_d() const -> int32_t {
  VERIFY_V3CBITSTREAM(m_ipdu_3d_range_d.has_value());
  return *m_ipdu_3d_range_d;
}

auto InterPatchDataUnit::ipdu_ref_index(int32_t value) -> InterPatchDataUnit & {
  m_ipdu_ref_index = value;
  return *this;
}

auto InterPatchDataUnit::ipdu_patch_index(int32_t value) -> InterPatchDataUnit & {
  m_ipdu_patch_index = value;
  return *this;
}

auto InterPatchDataUnit::ipdu_2d_pos_x(int32_t value) -> InterPatchDataUnit & {
  m_ipdu_2d_pos_x = value;
  return *this;
}

auto InterPatchDataUnit::ipdu_2d_pos_y(int32_t value) -> InterPatchDataUnit & {
  m_ipdu_2d_pos_y = value;
  return *this;
}

auto InterPatchDataUnit::ipdu_2d_delta_size_x(int32_t value) -> InterPatchDataUnit & {
  m_ipdu_2d_delta_size_x = value;
  return *this;
}

auto InterPatchDataUnit::ipdu_2d_delta_size_y(int32_t value) -> InterPatchDataUnit & {
  m_ipdu_2d_delta_size_y = value;
  return *this;
}

auto InterPatchDataUnit::ipdu_3d_offset_u(int32_t value) -> InterPatchDataUnit & {
  m_ipdu_3d_offset_u = value;
  return *this;
}

auto InterPatchDataUnit::ipdu_3d_offset_v(int32_t value) -> InterPatchDataUnit & {
  m_ipdu_3d_offset_v = value;
  return *this;
}

auto InterPatchDataUnit::ipdu_3d_offset_d(int32_t value) -> InterPatchDataUnit & {
  m_ipdu_3d_offset_d = value;
  return *this;
}

auto InterPatchDataUnit::ipdu_3d_range_d(int32_t value) -> InterPatchDataUnit & {
  m_ipdu_3d_range_d = value;
  return *this;
}

auto InterPatchDataUnit::printTo(std::ostream &stream, uint32_t tileId, size_t patchIdx) const
    -> std::ostream & {
  if (m_ipdu_ref_index) {
    TMIV_FMT::print(stream, "ipdu_ref_index[ {} ][ {} ]={}\n", tileId, patchIdx, *m_ipdu_ref_index);
  }
  TMIV_FMT::print(stream, "ipdu_patch_index[ {} ][ {} ]={}\n", tileId, patchIdx,
                  m_ipdu_patch_index);
  TMIV_FMT::print(stream, "ipdu_2d_pos_x[ {} ][ {} ]={}\n", tileId, patchIdx, m_ipdu_2d_pos_x);
  TMIV_FMT::print(stream, "ipdu_2d_pos_y[ {} ][ {} ]={}\n", tileId, patchIdx, m_ipdu_2d_pos_y);
  TMIV_FMT::print(stream, "ipdu_2d_delta_size_x[ {} ][ {} ]={}\n", tileId, patchIdx,
                  m_ipdu_2d_delta_size_x);
  TMIV_FMT::print(stream, "ipdu_2d_delta_size_y[ {} ][ {} ]={}\n", tileId, patchIdx,
                  m_ipdu_2d_delta_size_y);
  TMIV_FMT::print(stream, "ipdu_3d_offset_u[ {} ][ {} ]={}\n", tileId, patchIdx,
                  m_ipdu_3d_offset_u);
  TMIV_FMT::print(stream, "ipdu_3d_offset_v[ {} ][ {} ]={}\n", tileId, patchIdx,
                  m_ipdu_3d_offset_v);
  TMIV_FMT::print(stream, "ipdu_3d_offset_d[ {} ][ {} ]={}\n", tileId, patchIdx,
                  m_ipdu_3d_offset_d);

  if (m_ipdu_3d_range_d) {
    TMIV_FMT::print(stream, "ipdu_3d_range_d[ {} ][ {} ]={}\n", tileId, patchIdx,
                    *m_ipdu_3d_range_d);
  }
  return stream;
}

auto InterPatchDataUnit::operator==(const InterPatchDataUnit &other) const -> bool {
  return m_ipdu_ref_index == other.m_ipdu_ref_index &&
         m_ipdu_patch_index == other.m_ipdu_patch_index &&
         m_ipdu_2d_pos_x == other.m_ipdu_2d_pos_x && m_ipdu_2d_pos_y == other.m_ipdu_2d_pos_y &&
         m_ipdu_2d_delta_size_x == other.m_ipdu_2d_delta_size_x &&
         m_ipdu_2d_delta_size_y == other.m_ipdu_2d_delta_size_y &&
         m_ipdu_3d_offset_u == other.m_ipdu_3d_offset_u &&
         m_ipdu_3d_offset_v == other.m_ipdu_3d_offset_v &&
         m_ipdu_3d_offset_d == other.m_ipdu_3d_offset_d &&
         m_ipdu_3d_range_d == other.m_ipdu_3d_range_d;
}

auto InterPatchDataUnit::operator!=(const InterPatchDataUnit &other) const -> bool {
  return !operator==(other);
}

auto InterPatchDataUnit::decodeFrom(Common::InputBitstream &bitstream,
                                    const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                    const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                                    const AtlasTileHeader &ath) -> InterPatchDataUnit {
  auto x = InterPatchDataUnit{};

  const auto &afps = afpsById(afpsV, ath.ath_atlas_frame_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps.afps_atlas_sequence_parameter_set_id());

  Common::logWarning("Assuming that NumRefIdxActive == 1");
  // if (NumRefIdxActive > 1)
  //   x.ipdu_ref_index(bitstream.getUExpGolomb<int32_t>());

  x.ipdu_patch_index(bitstream.getSExpGolomb<int32_t>());
  x.ipdu_2d_pos_x(bitstream.getSExpGolomb<int32_t>());
  x.ipdu_2d_pos_y(bitstream.getSExpGolomb<int32_t>());
  x.ipdu_2d_delta_size_x(bitstream.getSExpGolomb<int32_t>());
  x.ipdu_2d_delta_size_y(bitstream.getSExpGolomb<int32_t>());
  x.ipdu_3d_offset_u(bitstream.getSExpGolomb<int32_t>());
  x.ipdu_3d_offset_v(bitstream.getSExpGolomb<int32_t>());
  x.ipdu_3d_offset_d(bitstream.getSExpGolomb<int32_t>());

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    x.ipdu_3d_range_d(bitstream.getSExpGolomb<int32_t>());
  }
  LIMITATION(!asps.asps_plr_enabled_flag());

  return x;
}

void InterPatchDataUnit::encodeTo(Common::OutputBitstream &bitstream,
                                  const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                  const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                                  const AtlasTileHeader &ath) const {
  const auto &afps = afpsById(afpsV, ath.ath_atlas_frame_parameter_set_id());
  const auto &asps = aspsById(aspsV, afps.afps_atlas_sequence_parameter_set_id());

  Common::logWarning("Assuming that NumRefIdxActive == 1");
  // if (NumRefIdxActive > 1)
  //   x.ipdu_ref_index(bitstream.getUExpGolomb<int32_t>());

  bitstream.putSExpGolomb(ipdu_patch_index());
  bitstream.putSExpGolomb(ipdu_2d_pos_x());
  bitstream.putSExpGolomb(ipdu_2d_pos_y());
  bitstream.putSExpGolomb(ipdu_2d_delta_size_x());
  bitstream.putSExpGolomb(ipdu_2d_delta_size_y());
  bitstream.putSExpGolomb(ipdu_3d_offset_u());
  bitstream.putSExpGolomb(ipdu_3d_offset_v());
  bitstream.putSExpGolomb(ipdu_3d_offset_d());

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    bitstream.putSExpGolomb(ipdu_3d_range_d());
  }
  LIMITATION(!asps.asps_plr_enabled_flag());
}

auto PatchInformationData::skip_patch_data_unit() const -> const SkipPatchDataUnit & {
  VERIFY_V3CBITSTREAM(m_skip_patch_data_unit.has_value());
  return *m_skip_patch_data_unit;
}

auto PatchInformationData::patch_data_unit() const -> const PatchDataUnit & {
  VERIFY_V3CBITSTREAM(m_patch_data_unit.has_value());
  return *m_patch_data_unit;
}

auto PatchInformationData::inter_patch_data_unit() const -> const InterPatchDataUnit & {
  VERIFY_V3CBITSTREAM(m_inter_patch_data_unit.has_value());
  return *m_inter_patch_data_unit;
}

auto PatchInformationData::skip_patch_data_unit() -> SkipPatchDataUnit & {
  if (!m_skip_patch_data_unit) {
    m_skip_patch_data_unit = SkipPatchDataUnit{};
  }
  return *m_skip_patch_data_unit;
}

auto PatchInformationData::patch_data_unit() -> PatchDataUnit & {
  if (!m_patch_data_unit) {
    m_patch_data_unit = PatchDataUnit{};
  }
  return *m_patch_data_unit;
}

auto PatchInformationData::inter_patch_data_unit() -> InterPatchDataUnit & {
  if (!m_inter_patch_data_unit) {
    m_inter_patch_data_unit = InterPatchDataUnit{};
  }
  return *m_inter_patch_data_unit;
}

auto PatchInformationData::printTo(std::ostream &stream, uint32_t tileId, size_t patchIdx) const
    -> std::ostream & {
  if (m_skip_patch_data_unit) {
    stream << *m_skip_patch_data_unit;
  }
  if (m_patch_data_unit) {
    m_patch_data_unit->printTo(stream, tileId, patchIdx);
  }
  if (m_inter_patch_data_unit) {
    m_inter_patch_data_unit->printTo(stream, tileId, patchIdx);
  }
  return stream;
}

auto PatchInformationData::operator==(const PatchInformationData &other) const noexcept -> bool {
  return m_skip_patch_data_unit == other.m_skip_patch_data_unit &&
         m_patch_data_unit == other.m_patch_data_unit &&
         m_inter_patch_data_unit == other.m_inter_patch_data_unit;
}

auto PatchInformationData::operator!=(const PatchInformationData &other) const noexcept -> bool {
  return !operator==(other);
}

auto PatchInformationData::decodeFrom(Common::InputBitstream &bitstream,
                                      const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                      const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                                      const AtlasTileHeader &ath, AtduPatchMode patchMode)
    -> PatchInformationData {
  auto x = PatchInformationData{};

  if (ath.ath_type() == AthType::P_TILE) {
    switch (patchMode) {
    case AtduPatchMode::P_MERGE:
      MIVBITSTREAM_ERROR("patchMode P_MERGE is not supported");
    case AtduPatchMode::P_SKIP:
      x.skip_patch_data_unit() = SkipPatchDataUnit::decodeFrom(bitstream);
      break;
    case AtduPatchMode::P_INTRA:
      x.patch_data_unit() = PatchDataUnit::decodeFrom(bitstream, aspsV, afpsV, ath);
      break;
    case AtduPatchMode::P_INTER:
      x.inter_patch_data_unit() = InterPatchDataUnit::decodeFrom(bitstream, aspsV, afpsV, ath);
      break;
    case AtduPatchMode::P_RAW:
      MIVBITSTREAM_ERROR("patchMode P_RAW is not supported");
    case AtduPatchMode::P_EOM:
      MIVBITSTREAM_ERROR("patchMode P_EOM is not supported");
    case AtduPatchMode::P_END:
      UNREACHABLE;
    }
  } else if (ath.ath_type() == AthType::I_TILE) {
    switch (patchMode) {
    case AtduPatchMode::I_INTRA:
      x.patch_data_unit() = PatchDataUnit::decodeFrom(bitstream, aspsV, afpsV, ath);
      break;
    case AtduPatchMode::I_RAW:
      MIVBITSTREAM_ERROR("patchMode I_RAW is not supported");
    case AtduPatchMode::I_EOM:
      MIVBITSTREAM_ERROR("patchMode I_EOM is not supported");
    case AtduPatchMode::I_END:
      UNREACHABLE;
    case AtduPatchMode::P_INTRA:
    case AtduPatchMode::P_RAW:
    case AtduPatchMode::P_EOM:
      V3CBITSTREAM_ERROR("Unexpected patchMode");
    }
  }
  return x;
}

void PatchInformationData::encodeTo(Common::OutputBitstream &bitstream,
                                    const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                    const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                                    const AtlasTileHeader &ath, AtduPatchMode patchMode) const {
  if (ath.ath_type() == AthType::P_TILE) {
    switch (patchMode) {
    case AtduPatchMode::P_MERGE:
      MIVBITSTREAM_ERROR("patchMode P_MERGE is not supported");
    case AtduPatchMode::P_SKIP:
      skip_patch_data_unit().encodeTo(bitstream);
      break;
    case AtduPatchMode::P_INTRA:
      patch_data_unit().encodeTo(bitstream, aspsV, afpsV, ath);
      break;
    case AtduPatchMode::P_INTER:
      inter_patch_data_unit().encodeTo(bitstream, aspsV, afpsV, ath);
      break;
    case AtduPatchMode::P_RAW:
      MIVBITSTREAM_ERROR("patchMode P_RAW is not supported");
    case AtduPatchMode::P_EOM:
      MIVBITSTREAM_ERROR("patchMode P_EOM is not supported");
    case AtduPatchMode::P_END:
      UNREACHABLE;
    }
  } else if (ath.ath_type() == AthType::I_TILE) {
    switch (patchMode) {
    case AtduPatchMode::I_INTRA:
      patch_data_unit().encodeTo(bitstream, aspsV, afpsV, ath);
      break;
    case AtduPatchMode::I_RAW:
      MIVBITSTREAM_ERROR("patchMode I_RAW is not supported");
    case AtduPatchMode::I_EOM:
      MIVBITSTREAM_ERROR("patchMode I_EOM is not supported");
    case AtduPatchMode::I_END:
      UNREACHABLE;
    case AtduPatchMode::P_INTRA:
    case AtduPatchMode::P_RAW:
    case AtduPatchMode::P_EOM:
      V3CBITSTREAM_ERROR("Unexpected patchMode");
    }
  }
}

auto AtlasTileDataUnit::skip_patch_data_unit() const -> const SkipPatchDataUnit & {
  VERIFY_V3CBITSTREAM(m_skip_patch_data_unit.has_value());
  return *m_skip_patch_data_unit;
}

auto AtlasTileDataUnit::atdu_patch_mode(size_t p) const -> AtduPatchMode {
  VERIFY_V3CBITSTREAM(p < m_atdu_patch_mode.size());
  return m_atdu_patch_mode[p];
}

auto AtlasTileDataUnit::patch_information_data(size_t p) const -> const PatchInformationData & {
  VERIFY_V3CBITSTREAM(p < m_patch_information_data.size());
  return m_patch_information_data[p];
}

auto AtlasTileDataUnit::skip_patch_data_unit() -> SkipPatchDataUnit & {
  if (!m_skip_patch_data_unit.has_value()) {
    m_skip_patch_data_unit = SkipPatchDataUnit{};
  }
  return *m_skip_patch_data_unit;
}

auto AtlasTileDataUnit::atdu_patch_mode(size_t p, AtduPatchMode value) -> AtlasTileDataUnit & {
  VERIFY_V3CBITSTREAM(!m_skip_patch_data_unit.has_value());
  VERIFY_V3CBITSTREAM(p <= m_atdu_patch_mode.size());

  if (p == m_atdu_patch_mode.size()) {
    m_atdu_patch_mode.push_back(value);
  }
  return *this;
}

auto AtlasTileDataUnit::patch_information_data(size_t p) -> PatchInformationData & {
  VERIFY_V3CBITSTREAM(!m_skip_patch_data_unit.has_value());
  VERIFY_V3CBITSTREAM(p <= m_patch_information_data.size());

  if (p == m_patch_information_data.size()) {
    return m_patch_information_data.emplace_back();
  }
  return m_patch_information_data[p];
}

auto AtlasTileDataUnit::atduTotalNumberOfPatches() const -> size_t {
  VERIFY_V3CBITSTREAM(!m_skip_patch_data_unit.has_value());
  VERIFY_V3CBITSTREAM(m_atdu_patch_mode.size() == 1 + m_patch_information_data.size());
  return m_patch_information_data.size();
}

auto AtlasTileDataUnit::printTo(std::ostream &stream, const AtlasTileHeader &ath) const
    -> std::ostream & {
  if (ath.ath_type() == AthType::SKIP_TILE) {
    // for( p = 0; p < RefAtduTotalNumPatches[ tileID ]; p++ )
    stream << skip_patch_data_unit();
  } else {
    auto p = size_t{};
    auto isEnd_ = bool{};
    do {
      TMIV_FMT::print(stream, "atdu_patch_mode[ {} ][ {} ]=", ath.ath_id(), p);
      MivBitstream::printTo(stream, atdu_patch_mode(p), ath.ath_type());
      TMIV_FMT::print(stream, "\n");
      isEnd_ = isEnd(ath.ath_type(), atdu_patch_mode(p));

      if (!isEnd_) {
        patch_information_data(p).printTo(stream, ath.ath_id(), p);
        ++p;
      }
    } while (!isEnd_);
    TMIV_FMT::print(stream, "AtduTotalNumPatches[ {} ]={}\n", ath.ath_id(), p);
  }
  return stream;
}

auto AtlasTileDataUnit::operator==(const AtlasTileDataUnit &other) const -> bool {
  if (m_skip_patch_data_unit != other.m_skip_patch_data_unit) {
    return false;
  }
  if (m_skip_patch_data_unit) {
    return true;
  }
  return m_atdu_patch_mode == other.m_atdu_patch_mode &&
         m_patch_information_data == other.m_patch_information_data;
}

auto AtlasTileDataUnit::operator!=(const AtlasTileDataUnit &other) const -> bool {
  return !operator==(other);
}

auto AtlasTileDataUnit::decodeFrom(Common::InputBitstream &bitstream,
                                   const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                   const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                                   const AtlasTileHeader &ath) -> AtlasTileDataUnit {
  auto x = AtlasTileDataUnit{};

  if (ath.ath_type() == AthType::SKIP_TILE) {
    // for( p = 0; p < RefAtduTotalNumPatches[ tileID ]; p++ )
    x.skip_patch_data_unit() = SkipPatchDataUnit::decodeFrom(bitstream);
  } else {
    auto p = size_t{};
    auto isEnd_ = bool{};
    do {
      x.atdu_patch_mode(p, bitstream.getUExpGolomb<AtduPatchMode>());
      isEnd_ = isEnd(ath.ath_type(), x.atdu_patch_mode(p));

      if (!isEnd_) {
        x.patch_information_data(p) =
            PatchInformationData::decodeFrom(bitstream, aspsV, afpsV, ath, x.atdu_patch_mode(p));
        ++p;
      }
    } while (!isEnd_);

    VERIFY_V3CBITSTREAM(x.atduTotalNumberOfPatches() == p);
  }
  return x;
}

void AtlasTileDataUnit::encodeTo(Common::OutputBitstream &bitstream,
                                 const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                 const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                                 const AtlasTileHeader &ath) const {
  if (ath.ath_type() == AthType::SKIP_TILE) {
    // for( p = 0; p < RefAtduTotalNumPatches[ tileID ]; p++ )
    skip_patch_data_unit().encodeTo(bitstream);
  } else {
    auto p = size_t{};
    auto isEnd_ = bool{};
    do {
      bitstream.putUExpGolomb(atdu_patch_mode(p));
      isEnd_ = isEnd(ath.ath_type(), atdu_patch_mode(p));

      if (!isEnd_) {
        patch_information_data(p).encodeTo(bitstream, aspsV, afpsV, ath, atdu_patch_mode(p));
        ++p;
      }
    } while (!isEnd_);
  }
}

auto AtlasTileDataUnit::isEnd(AthType athType, AtduPatchMode patchMode) -> bool {
  return (athType == AthType::P_TILE && patchMode == AtduPatchMode::P_END) ||
         (athType == AthType::I_TILE && patchMode == AtduPatchMode::I_END);
}

auto operator<<(std::ostream &stream, const AtlasTileLayerRBSP &x) -> std::ostream & {
  stream << x.atlas_tile_header();
  x.atlas_tile_data_unit().printTo(stream, x.atlas_tile_header());
  return stream;
}

auto AtlasTileLayerRBSP::operator==(const AtlasTileLayerRBSP &other) const noexcept -> bool {
  return atlas_tile_header() == other.atlas_tile_header() &&
         atlas_tile_data_unit() == other.atlas_tile_data_unit();
}

auto AtlasTileLayerRBSP::operator!=(const AtlasTileLayerRBSP &other) const noexcept -> bool {
  return !operator==(other);
}

auto AtlasTileLayerRBSP::decodeFrom(std::istream &stream, const NalUnitHeader &nuh,
                                    const std::vector<AtlasSequenceParameterSetRBSP> &asps,
                                    const std::vector<AtlasFrameParameterSetRBSP> &afps)
    -> AtlasTileLayerRBSP {
  Common::InputBitstream bitstream{stream};

  auto atl = AtlasTileLayerRBSP{};
  atl.atlas_tile_header() = AtlasTileHeader::decodeFrom(bitstream, nuh, asps, afps);
  atl.atlas_tile_data_unit() =
      AtlasTileDataUnit::decodeFrom(bitstream, asps, afps, atl.atlas_tile_header());
  bitstream.rbspTrailingBits();

  return atl;
}

void AtlasTileLayerRBSP::encodeTo(std::ostream &stream, const NalUnitHeader &nuh,
                                  const std::vector<AtlasSequenceParameterSetRBSP> &asps,
                                  const std::vector<AtlasFrameParameterSetRBSP> &afps) const {
  Common::OutputBitstream bitstream{stream};

  atlas_tile_header().encodeTo(bitstream, nuh, asps, afps);
  atlas_tile_data_unit().encodeTo(bitstream, asps, afps, atlas_tile_header());
  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream
