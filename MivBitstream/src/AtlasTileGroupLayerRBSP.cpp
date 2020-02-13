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

#include <TMIV/MivBitstream/AtlasTileGroupLayerRBSP.h>

#include <TMIV/Common/Common.h>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, AtghType x) -> ostream & {
  switch (x) {
  case AtghType::P_TILE_GRP:
    return stream << "P_TILE_GRP";
  case AtghType::I_TILE_GRP:
    return stream << "I_TILE_GRP";
  case AtghType::SKIP_TILE_GRP:
    return stream << "SKIP_TILE_GRP";
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

auto printTo(ostream &stream, AtgduPatchMode x, AtghType atgh_type) -> ostream & {
  switch (atgh_type) {
  case AtghType::I_TILE_GRP:
    switch (x) {
    case AtgduPatchMode::I_INTRA:
      return stream << "I_INTRA";
    case AtgduPatchMode::I_RAW:
      return stream << "I_RAW";
    case AtgduPatchMode::I_EOM:
      return stream << "I_EOM";
    case AtgduPatchMode::I_END:
      return stream << "I_END";
    default:
      return stream << "[unknown:" << int(x) << "]";
    }
  case AtghType::P_TILE_GRP:
    switch (x) {
    case AtgduPatchMode::P_SKIP:
      return stream << "P_SKIP";
    case AtgduPatchMode::P_MERGE:
      return stream << "P_MERGE";
    case AtgduPatchMode::P_INTER:
      return stream << "P_INTER";
    case AtgduPatchMode::P_INTRA:
      return stream << "P_INTRA";
    case AtgduPatchMode::P_RAW:
      return stream << "P_RAW";
    case AtgduPatchMode::P_EOM:
      return stream << "P_EOM";
    case AtgduPatchMode::P_END:
      return stream << "P_END";
    default:
      return stream << "[unknown:" << int(x) << "]";
    }
  case AtghType::SKIP_TILE_GRP:
    switch (x) {
    case AtgduPatchMode::P_SKIP:
      return stream << "P_SKIP";
    default:
      return stream << "[unknown:" << int(x) << "]";
    }
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto AtlasTileGroupHeader::atgh_adaptation_parameter_set_id() const noexcept -> uint8_t {
  VERIFY_VPCCBITSTREAM(m_atgh_adaptation_parameter_set_id.has_value());
  return *m_atgh_adaptation_parameter_set_id;
}

auto AtlasTileGroupHeader::atgh_patch_size_x_info_quantizer() const noexcept -> uint8_t {
  VERIFY_VPCCBITSTREAM(atgh_type() != AtghType::SKIP_TILE_GRP);
  return m_atgh_patch_size_x_info_quantizer;
}

auto AtlasTileGroupHeader::atgh_patch_size_y_info_quantizer() const noexcept -> uint8_t {
  VERIFY_VPCCBITSTREAM(atgh_type() != AtghType::SKIP_TILE_GRP);
  return m_atgh_patch_size_y_info_quantizer;
}

auto AtlasTileGroupHeader::atgh_patch_size_x_info_quantizer(const uint8_t value) noexcept
    -> AtlasTileGroupHeader & {
  VERIFY_VPCCBITSTREAM(atgh_type() != AtghType::SKIP_TILE_GRP);
  m_atgh_patch_size_x_info_quantizer = value;
  return *this;
}

auto AtlasTileGroupHeader::atgh_patch_size_y_info_quantizer(const uint8_t value) noexcept
    -> AtlasTileGroupHeader & {
  VERIFY_VPCCBITSTREAM(atgh_type() != AtghType::SKIP_TILE_GRP);
  m_atgh_patch_size_y_info_quantizer = value;
  return *this;
}

auto operator<<(ostream &stream, const AtlasTileGroupHeader &x) -> ostream & {
  stream << "atgh_atlas_frame_parameter_set_id=" << int(x.atgh_atlas_frame_parameter_set_id());
  if (x.m_atgh_adaptation_parameter_set_id) {
    stream << "\natgh_adaptation_parameter_set_id=" << int(*x.m_atgh_adaptation_parameter_set_id);
  }
  stream << "\natgh_address=" << int(x.atgh_address()) << "\natgh_type=" << x.atgh_type()
         << "\natgh_atlas_frm_order_cnt_lsb=" << int(x.atgh_atlas_frm_order_cnt_lsb()) << '\n';
  if (x.atgh_type() != AtghType::SKIP_TILE_GRP) {
    stream << "atgh_patch_size_x_info_quantizer=" << int(x.atgh_patch_size_x_info_quantizer())
           << "\natgh_patch_size_y_info_quantizer=" << int(x.atgh_patch_size_y_info_quantizer())
           << '\n';
  }
  return stream;
}

auto AtlasTileGroupHeader::decodeFrom(InputBitstream &bitstream,
                                      const vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                      const vector<AtlasFrameParameterSetRBSP> &afpsV)
    -> AtlasTileGroupHeader {
  auto x = AtlasTileGroupHeader{};

  x.atgh_atlas_frame_parameter_set_id(uint8_t(bitstream.getUExpGolomb()));
  VERIFY_VPCCBITSTREAM(x.atgh_atlas_frame_parameter_set_id() <= 63);
  VERIFY_VPCCBITSTREAM(x.atgh_atlas_frame_parameter_set_id() < afpsV.size());
  const auto &afps = afpsV[x.atgh_atlas_frame_parameter_set_id()];

  VERIFY_VPCCBITSTREAM(afps.afps_atlas_sequence_parameter_set_id() < aspsV.size());
  const auto &asps = aspsV[afps.afps_atlas_sequence_parameter_set_id()];

  if (!afps.afps_fixed_camera_model_flag()) {
    x.atgh_adaptation_parameter_set_id(uint8_t(bitstream.getUExpGolomb()));
  }

  VERIFY_MIVBITSTREAM(afps.atlas_frame_tile_information().afti_single_tile_in_atlas_frame_flag());
  x.atgh_address(0);

  x.atgh_type(AtghType(uint8_t(bitstream.getUExpGolomb())));
  VERIFY_MIVBITSTREAM(x.atgh_type() == AtghType::I_TILE_GRP ||
                      x.atgh_type() == AtghType::SKIP_TILE_GRP);

  x.atgh_atlas_frm_order_cnt_lsb(
      uint8_t(bitstream.readBits(asps.asps_log2_max_atlas_frame_order_cnt_lsb())));

  // Only intra coding (for now)
  VERIFY_MIVBITSTREAM(asps.asps_num_ref_atlas_frame_lists_in_asps() == 1);
  const auto atgh_ref_atlas_frame_list_sps_flag = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(atgh_ref_atlas_frame_list_sps_flag);
  VERIFY_MIVBITSTREAM(asps.ref_list_struct(0).num_ref_entries() == 0);

  if (x.atgh_type() != AtghType::SKIP_TILE_GRP) {
    VERIFY_MIVBITSTREAM(!asps.asps_normal_axis_limits_quantization_enabled_flag());
    static_assert(x.atgh_pos_min_z_quantizer() == 0);
    static_assert(x.atgh_pos_max_z_quantizer() == 0);

    if (asps.asps_patch_size_quantizer_present_flag()) {
      x.atgh_patch_size_x_info_quantizer(uint8_t(bitstream.readBits(3)));
      VERIFY_VPCCBITSTREAM(x.atgh_patch_size_x_info_quantizer() <=
                           asps.asps_log2_patch_packing_block_size());

      x.atgh_patch_size_y_info_quantizer(uint8_t(bitstream.readBits(3)));
      VERIFY_VPCCBITSTREAM(x.atgh_patch_size_y_info_quantizer() <=
                           asps.asps_log2_patch_packing_block_size());
    } else {
      x.atgh_patch_size_x_info_quantizer(asps.asps_log2_patch_packing_block_size());
      x.atgh_patch_size_y_info_quantizer(asps.asps_log2_patch_packing_block_size());
    }

    VERIFY_MIVBITSTREAM(!afps.afps_raw_3d_pos_bit_count_explicit_mode_flag());
  }

  bitstream.byteAlign();

  return x;
}

void AtlasTileGroupHeader::encodeTo(OutputBitstream &bitstream,
                                    const vector<AtlasSequenceParameterSetRBSP> &aspsV,
                                    const vector<AtlasFrameParameterSetRBSP> &afpsV) const {
  VERIFY_VPCCBITSTREAM(atgh_atlas_frame_parameter_set_id() <= 63);
  VERIFY_VPCCBITSTREAM(atgh_atlas_frame_parameter_set_id() < afpsV.size());
  bitstream.putUExpGolomb(atgh_atlas_frame_parameter_set_id());
  const auto &afps = afpsV[atgh_atlas_frame_parameter_set_id()];

  VERIFY_VPCCBITSTREAM(afps.afps_atlas_sequence_parameter_set_id() < aspsV.size());
  const auto &asps = aspsV[afps.afps_atlas_sequence_parameter_set_id()];

  if (afps.afps_fixed_camera_model_flag()) {
    VERIFY_VPCCBITSTREAM(!m_atgh_adaptation_parameter_set_id.has_value());
  } else {
    bitstream.putUExpGolomb(atgh_adaptation_parameter_set_id());
  }

  VERIFY_MIVBITSTREAM(afps.atlas_frame_tile_information().afti_single_tile_in_atlas_frame_flag());
  VERIFY_VPCCBITSTREAM(atgh_address() == 0);

  VERIFY_MIVBITSTREAM(atgh_type() == AtghType::I_TILE_GRP ||
                      atgh_type() == AtghType::SKIP_TILE_GRP);
  bitstream.putUExpGolomb(unsigned(atgh_type()));

  bitstream.writeBits(atgh_atlas_frm_order_cnt_lsb(),
                      asps.asps_log2_max_atlas_frame_order_cnt_lsb());

  VERIFY_MIVBITSTREAM(asps.asps_num_ref_atlas_frame_lists_in_asps() == 1);
  constexpr auto atgh_ref_atlas_frame_list_sps_flag = true;
  bitstream.putFlag(atgh_ref_atlas_frame_list_sps_flag);

  if (atgh_type() != AtghType::SKIP_TILE_GRP) {
    VERIFY_MIVBITSTREAM(!asps.asps_normal_axis_limits_quantization_enabled_flag());

    if (asps.asps_patch_size_quantizer_present_flag()) {
      VERIFY_VPCCBITSTREAM(atgh_patch_size_x_info_quantizer() <=
                           asps.asps_log2_patch_packing_block_size());
      bitstream.writeBits(atgh_patch_size_x_info_quantizer(), 3);

      VERIFY_VPCCBITSTREAM(atgh_patch_size_y_info_quantizer() <=
                           asps.asps_log2_patch_packing_block_size());
      bitstream.writeBits(atgh_patch_size_y_info_quantizer(), 3);
    } else {
      VERIFY_VPCCBITSTREAM(atgh_patch_size_x_info_quantizer() ==
                           asps.asps_log2_patch_packing_block_size());
      VERIFY_VPCCBITSTREAM(atgh_patch_size_y_info_quantizer() ==
                           asps.asps_log2_patch_packing_block_size());
    }

    VERIFY_MIVBITSTREAM(!afps.afps_raw_3d_pos_bit_count_explicit_mode_flag());
  }

  bitstream.byteAlign();
}

auto operator<<(ostream &stream, const SkipPatchDataUnit & /* x */) -> ostream & { return stream; }

auto PatchDataUnit::printTo(ostream &stream, size_t patchIdx) const -> ostream & {
  stream << "pdu_2d_pos_x( " << patchIdx << " )=" << pdu_2d_pos_x() << "\npdu_2d_pos_y( "
         << patchIdx << " )=" << pdu_2d_pos_y() << "\npdu_2d_size_x( " << patchIdx
         << " )=" << pdu_2d_size_x() << "\npdu_2d_size_y( " << patchIdx << " )=" << pdu_2d_size_y()
         << "\npdu_3d_pos_x( " << patchIdx << " )=" << pdu_3d_pos_x() << "\npdu_3d_pos_y( "
         << patchIdx << " )=" << pdu_3d_pos_y() << "\npdu_3d_pos_min_z( " << patchIdx
         << " )=" << pdu_3d_pos_min_z() << '\n';
  if (pdu_3d_pos_delta_max_z()) {
    stream << "pdu_3d_pos_delta_max_z( " << patchIdx << " )=" << *pdu_3d_pos_delta_max_z() << '\n';
  }
  stream << "pdu_projection_id( " << patchIdx << " )=" << pdu_projection_id()
         << "\npdu_orientation_index( " << patchIdx << " )=" << pdu_orientation_index() << '\n';
  if (pdu_lod()) {
    stream << "pdu_lod( " << patchIdx << " )=" << *pdu_lod() << '\n';
  }
  return stream;
}

auto PatchDataUnit::decodeFrom(InputBitstream &bitstream, const VpccUnitHeader &vuh,
                               const VpccParameterSet &vps,
                               const vector<AtlasSequenceParameterSetRBSP> &aspsV,
                               const vector<AtlasFrameParameterSetRBSP> &afpsV,
                               const AtlasTileGroupHeader &atgh, const PatchDataUnit *previous)
    -> PatchDataUnit {
  auto x = PatchDataUnit{};

  VERIFY_VPCCBITSTREAM(atgh.atgh_atlas_frame_parameter_set_id() < afpsV.size());
  const auto &afps = afpsV[atgh.atgh_atlas_frame_parameter_set_id()];

  VERIFY_VPCCBITSTREAM(afps.afps_atlas_sequence_parameter_set_id() < aspsV.size());
  const auto &asps = aspsV[afps.afps_atlas_sequence_parameter_set_id()];

  const auto pdu_projection_id_num_bits =
      vps.overridePduProjectionIdNumBits()
          ? *vps.overridePduProjectionIdNumBits()
          : (asps.asps_45degree_projection_patch_present_flag() ? 5U : 3U);
  x.pdu_projection_id(uint16_t(bitstream.readBits(pdu_projection_id_num_bits)));

  x.pdu_2d_pos_x(uint32_t(bitstream.readBits(afps.afps_2d_pos_x_bit_count())));
  x.pdu_2d_pos_y(uint32_t(bitstream.readBits(afps.afps_2d_pos_y_bit_count())));

  VERIFY_VPCCBITSTREAM(x.pdu_2d_pos_x() < asps.asps_frame_width());
  VERIFY_VPCCBITSTREAM(x.pdu_2d_pos_y() < asps.asps_frame_height());

  if (previous != nullptr) {
    x.pdu_2d_size_x(uint32_t(bitstream.getUExpGolomb() + previous->pdu_2d_size_x()));
    x.pdu_2d_size_y(uint32_t(bitstream.getUExpGolomb() + previous->pdu_2d_size_y()));
  } else {
    x.pdu_2d_size_x(uint32_t(bitstream.getUExpGolomb()));
    x.pdu_2d_size_y(uint32_t(bitstream.getUExpGolomb()));

    VERIFY_VPCCBITSTREAM(0 < x.pdu_2d_size_x());
    VERIFY_VPCCBITSTREAM(0 < x.pdu_2d_size_y());
  }

  VERIFY_VPCCBITSTREAM(x.pdu_2d_pos_x() + x.pdu_2d_size_x() <= asps.asps_frame_width());
  VERIFY_VPCCBITSTREAM(x.pdu_2d_pos_y() + x.pdu_2d_size_y() <= asps.asps_frame_height());

  x.pdu_3d_pos_x(uint32_t(bitstream.readBits(afps.afps_3d_pos_x_bit_count())));
  x.pdu_3d_pos_y(uint32_t(bitstream.readBits(afps.afps_3d_pos_y_bit_count())));

  VERIFY_VPCCBITSTREAM(vuh.vuh_unit_type() == VuhUnitType::VPCC_AD);
  const auto &gi = vps.geometry_information(vuh.vuh_atlas_id());

  const auto pdu_3d_pos_min_z_num_bits = gi.gi_geometry_3d_coordinates_bitdepth() -
                                         atgh.atgh_pos_min_z_quantizer() +
                                         int(x.pdu_projection_id() > 5);
  x.pdu_3d_pos_min_z(uint32_t(bitstream.readBits(pdu_3d_pos_min_z_num_bits)));

  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    const auto pdu_3d_pos_max_z_num_bits = gi.gi_geometry_3d_coordinates_bitdepth() -
                                           atgh.atgh_pos_max_z_quantizer() +
                                           int(x.pdu_projection_id() > 5);
    x.pdu_3d_pos_delta_max_z({uint32_t(bitstream.readBits(pdu_3d_pos_max_z_num_bits))});
  }

  const auto pdu_orientation_index_num_bits = asps.asps_use_eight_orientations_flag() ? 3 : 1;
  x.pdu_orientation_index(
      FlexiblePatchOrientation(bitstream.readBits(pdu_orientation_index_num_bits)));

  if (afps.afps_lod_bit_count() > 0) {
    x.pdu_lod({uint32_t(bitstream.readBits(afps.afps_lod_bit_count()))});
  }

  VERIFY_MIVBITSTREAM(!asps.asps_point_local_reconstruction_enabled_flag());

  return x;
}

void PatchDataUnit::encodeTo(OutputBitstream &bitstream, const VpccUnitHeader &vuh,
                             const VpccParameterSet &vps,
                             const vector<AtlasSequenceParameterSetRBSP> &aspsV,
                             const vector<AtlasFrameParameterSetRBSP> &afpsV,
                             const AtlasTileGroupHeader &atgh,
                             const PatchDataUnit *previous) const {
  VERIFY_VPCCBITSTREAM(atgh.atgh_atlas_frame_parameter_set_id() < afpsV.size());
  const auto &afps = afpsV[atgh.atgh_atlas_frame_parameter_set_id()];

  VERIFY_VPCCBITSTREAM(afps.afps_atlas_sequence_parameter_set_id() < aspsV.size());
  const auto &asps = aspsV[afps.afps_atlas_sequence_parameter_set_id()];

  const auto pdu_projection_id_num_bits =
      vps.overridePduProjectionIdNumBits()
          ? *vps.overridePduProjectionIdNumBits()
          : asps.asps_45degree_projection_patch_present_flag() ? 5U : 3U;
  VERIFY_VPCCBITSTREAM((pdu_projection_id() >> pdu_projection_id_num_bits) == 0);
  bitstream.writeBits(pdu_projection_id(), pdu_projection_id_num_bits);

  VERIFY_VPCCBITSTREAM(pdu_2d_pos_x() + pdu_2d_size_x() <= asps.asps_frame_width());
  VERIFY_VPCCBITSTREAM(pdu_2d_pos_y() + pdu_2d_size_y() <= asps.asps_frame_height());

  VERIFY_VPCCBITSTREAM((pdu_2d_pos_x() >> afps.afps_2d_pos_x_bit_count()) == 0);
  VERIFY_VPCCBITSTREAM((pdu_2d_pos_y() >> afps.afps_2d_pos_y_bit_count()) == 0);

  bitstream.writeBits(pdu_2d_pos_x(), afps.afps_2d_pos_x_bit_count());
  bitstream.writeBits(pdu_2d_pos_y(), afps.afps_2d_pos_y_bit_count());

  if (previous != nullptr) {
    VERIFY_VPCCBITSTREAM(previous->pdu_2d_size_x() <= pdu_2d_size_x());
    VERIFY_VPCCBITSTREAM(previous->pdu_2d_size_y() <= pdu_2d_size_y());

    bitstream.putUExpGolomb(pdu_2d_size_x() - previous->pdu_2d_size_x());
    bitstream.putUExpGolomb(pdu_2d_size_y() - previous->pdu_2d_size_y());
  } else {
    VERIFY_VPCCBITSTREAM(0 < pdu_2d_size_x());
    VERIFY_VPCCBITSTREAM(0 < pdu_2d_size_y());

    bitstream.putUExpGolomb(pdu_2d_size_x());
    bitstream.putUExpGolomb(pdu_2d_size_y());
  }

  VERIFY_VPCCBITSTREAM((pdu_3d_pos_x() >> afps.afps_3d_pos_x_bit_count()) == 0);
  VERIFY_VPCCBITSTREAM((pdu_3d_pos_y() >> afps.afps_3d_pos_y_bit_count()) == 0);

  bitstream.writeBits(pdu_3d_pos_x(), afps.afps_3d_pos_x_bit_count());
  bitstream.writeBits(pdu_3d_pos_y(), afps.afps_3d_pos_y_bit_count());

  VERIFY_VPCCBITSTREAM(vuh.vuh_unit_type() == VuhUnitType::VPCC_AD);
  const auto &gi = vps.geometry_information(vuh.vuh_atlas_id());

  const auto pdu_3d_pos_min_z_num_bits = gi.gi_geometry_3d_coordinates_bitdepth() -
                                         atgh.atgh_pos_min_z_quantizer() +
                                         int(pdu_projection_id() > 5);
  VERIFY_VPCCBITSTREAM((pdu_3d_pos_min_z() >> pdu_3d_pos_min_z_num_bits) == 0);
  bitstream.writeBits(pdu_3d_pos_min_z(), pdu_3d_pos_min_z_num_bits);

  VERIFY_VPCCBITSTREAM(!pdu_3d_pos_delta_max_z() ==
                       !asps.asps_normal_axis_max_delta_value_enabled_flag());
  if (asps.asps_normal_axis_max_delta_value_enabled_flag()) {
    const auto pdu_3d_pos_max_z_num_bits = gi.gi_geometry_3d_coordinates_bitdepth() -
                                           atgh.atgh_pos_max_z_quantizer() +
                                           int(pdu_projection_id() > 5);
    VERIFY_VPCCBITSTREAM((*pdu_3d_pos_delta_max_z() >> pdu_3d_pos_max_z_num_bits) == 0);
    bitstream.writeBits(*pdu_3d_pos_delta_max_z(), pdu_3d_pos_max_z_num_bits);
  }

  if (asps.asps_use_eight_orientations_flag()) {
    bitstream.writeBits(unsigned(pdu_orientation_index()), 3);
  } else {
    VERIFY_VPCCBITSTREAM(pdu_orientation_index() == FlexiblePatchOrientation::FPO_NULL ||
                         pdu_orientation_index() == FlexiblePatchOrientation::FPO_SWAP);
    bitstream.writeBits(unsigned(pdu_orientation_index()), 1);
  }

  VERIFY_VPCCBITSTREAM(!pdu_lod() == (afps.afps_lod_bit_count() == 0));
  if (afps.afps_lod_bit_count() > 0) {
    VERIFY_VPCCBITSTREAM((*pdu_lod() >> afps.afps_lod_bit_count()) == 0);
    bitstream.writeBits(*pdu_lod(), afps.afps_lod_bit_count());
  }

  VERIFY_MIVBITSTREAM(!asps.asps_point_local_reconstruction_enabled_flag());
}

auto PatchInformationData::skip_patch_data_unit() const noexcept -> const SkipPatchDataUnit & {
  VERIFY_VPCCBITSTREAM(holds_alternative<SkipPatchDataUnit>(m_data));
  return *get_if<SkipPatchDataUnit>(&m_data);
}

auto PatchInformationData::patch_data_unit() const noexcept -> const PatchDataUnit & {
  VERIFY_VPCCBITSTREAM(holds_alternative<PatchDataUnit>(m_data));
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

auto PatchInformationData::decodeFrom(InputBitstream &bitstream, const VpccUnitHeader &vuh,
                                      const VpccParameterSet &vps,
                                      const vector<AtlasSequenceParameterSetRBSP> &asps,
                                      const vector<AtlasFrameParameterSetRBSP> &afps,
                                      const AtlasTileGroupHeader &atgh, AtgduPatchMode patchMode,
                                      const PatchDataUnit *previous) -> PatchInformationData {
  if (atgh.atgh_type() == AtghType::I_TILE_GRP) {
    VERIFY_VPCCBITSTREAM(patchMode == AtgduPatchMode::I_INTRA);
    return PatchInformationData{
        PatchDataUnit::decodeFrom(bitstream, vuh, vps, asps, afps, atgh, previous)};
  }
  if (atgh.atgh_type() == AtghType::SKIP_TILE_GRP) {
    VERIFY_VPCCBITSTREAM(patchMode == AtgduPatchMode::P_SKIP);
    return PatchInformationData{SkipPatchDataUnit::decodeFrom(bitstream)};
  }
  VPCCBITSTREAM_ERROR("Unknown or unsupported tile group/patch mode combination");
}

void PatchInformationData::encodeTo(OutputBitstream &bitstream, const VpccUnitHeader &vuh,
                                    const VpccParameterSet &vps,
                                    const vector<AtlasSequenceParameterSetRBSP> &asps,
                                    const vector<AtlasFrameParameterSetRBSP> &afps,
                                    const AtlasTileGroupHeader &atgh, AtgduPatchMode patchMode,
                                    const PatchDataUnit *previous) const {
  if (atgh.atgh_type() == AtghType::I_TILE_GRP) {
    VERIFY_VPCCBITSTREAM(patchMode == AtgduPatchMode::I_INTRA);
    return patch_data_unit().encodeTo(bitstream, vuh, vps, asps, afps, atgh, previous);
  }
  if (atgh.atgh_type() == AtghType::SKIP_TILE_GRP) {
    VERIFY_VPCCBITSTREAM(patchMode == AtgduPatchMode::P_SKIP);
    return skip_patch_data_unit().encodeTo(bitstream);
  }
  VPCCBITSTREAM_ERROR("Unknown or unsupported tile group/patch mode combination");
}

auto AtlasTileGroupDataUnit::atgduTotalNumberOfPatches() const noexcept -> size_t {
  return m_vector.size();
}

auto AtlasTileGroupDataUnit::atgdu_patch_mode(size_t p) const -> AtgduPatchMode {
  VERIFY_VPCCBITSTREAM(p < m_vector.size());
  return m_vector[p].first;
}

auto AtlasTileGroupDataUnit::patch_information_data(size_t p) const
    -> const PatchInformationData & {
  VERIFY_VPCCBITSTREAM(p < m_vector.size());
  return m_vector[p].second;
}

auto AtlasTileGroupDataUnit::printTo(ostream &stream, AtghType atgh_type) const -> ostream & {
  visit([&](const auto p, const AtgduPatchMode patch_mode,
            const PatchInformationData &patch_information_data) {
    stream << "atgdu_patch_mode[ " << p << " ]=";
    MivBitstream::printTo(stream, patch_mode, atgh_type) << '\n';
    patch_information_data.printTo(stream, p);
  });
  return stream;
}

auto AtlasTileGroupDataUnit::operator==(const AtlasTileGroupDataUnit &other) const -> bool {
  return m_vector == other.m_vector;
}

auto AtlasTileGroupDataUnit::operator!=(const AtlasTileGroupDataUnit &other) const -> bool {
  return !operator==(other);
}

auto AtlasTileGroupDataUnit::decodeFrom(InputBitstream &bitstream, const VpccUnitHeader &vuh,
                                        const VpccParameterSet &vps,
                                        const vector<AtlasSequenceParameterSetRBSP> &asps,
                                        const vector<AtlasFrameParameterSetRBSP> &afps,
                                        const AtlasTileGroupHeader &atgh)
    -> AtlasTileGroupDataUnit {
  VERIFY_VPCCBITSTREAM(atgh.atgh_type() == AtghType::I_TILE_GRP ||
                       atgh.atgh_type() == AtghType::P_TILE_GRP);
  VERIFY_MIVBITSTREAM(atgh.atgh_type() == AtghType::I_TILE_GRP);

  auto x = AtlasTileGroupDataUnit::Vector{};
  const PatchDataUnit *previous = nullptr;
  auto patch_mode = AtgduPatchMode(bitstream.getUExpGolomb());

  while (patch_mode != AtgduPatchMode::I_END) {
    x.emplace_back(patch_mode, PatchInformationData::decodeFrom(bitstream, vuh, vps, asps, afps,
                                                                atgh, patch_mode, previous));
    VERIFY_MIVBITSTREAM(patch_mode == AtgduPatchMode::I_INTRA);
    previous = &x.back().second.patch_data_unit();
    patch_mode = AtgduPatchMode(bitstream.getUExpGolomb());
  }

  bitstream.byteAlign();
  return AtlasTileGroupDataUnit{x};
}

void AtlasTileGroupDataUnit::encodeTo(OutputBitstream &bitstream, const VpccUnitHeader &vuh,
                                      const VpccParameterSet &vps,
                                      const vector<AtlasSequenceParameterSetRBSP> &asps,
                                      const vector<AtlasFrameParameterSetRBSP> &afps,
                                      const AtlasTileGroupHeader &atgh) const {
  VERIFY_VPCCBITSTREAM(atgh.atgh_type() == AtghType::I_TILE_GRP ||
                       atgh.atgh_type() == AtghType::P_TILE_GRP);
  VERIFY_MIVBITSTREAM(atgh.atgh_type() == AtghType::I_TILE_GRP);

  const PatchDataUnit *previous = nullptr;

  visit([&](const auto /* p */, const AtgduPatchMode patch_mode,
            const PatchInformationData &patch_information_data) {
    bitstream.putUExpGolomb(int(patch_mode));
    patch_information_data.encodeTo(bitstream, vuh, vps, asps, afps, atgh, patch_mode, previous);
    if (patch_mode == AtgduPatchMode::I_INTRA) {
      previous = &patch_information_data.patch_data_unit();
    }
  });

  bitstream.putUExpGolomb(int(AtgduPatchMode::I_END));
  bitstream.byteAlign();
}

auto AtlasTileGroupLayerRBSP::atlas_tile_group_data_unit() const noexcept
    -> const AtlasTileGroupDataUnit & {
  VERIFY_VPCCBITSTREAM(m_atlas_tile_group_data_unit);
  return *m_atlas_tile_group_data_unit;
}

auto operator<<(ostream &stream, const AtlasTileGroupLayerRBSP &x) -> ostream & {
  stream << x.atlas_tile_group_header();
  if (x.atlas_tile_group_header().atgh_type() != AtghType::SKIP_TILE_GRP) {
    x.atlas_tile_group_data_unit().printTo(stream, x.atlas_tile_group_header().atgh_type());
  }
  return stream;
}

auto AtlasTileGroupLayerRBSP::decodeFrom(istream &stream, const VpccUnitHeader &vuh,
                                         const VpccParameterSet &vps,
                                         const vector<AtlasSequenceParameterSetRBSP> &asps,
                                         const vector<AtlasFrameParameterSetRBSP> &afps)
    -> AtlasTileGroupLayerRBSP {
  InputBitstream bitstream{stream};

  const auto atgh = AtlasTileGroupHeader::decodeFrom(bitstream, asps, afps);
  auto atgl = AtlasTileGroupLayerRBSP{atgh};

  if (atgh.atgh_type() != AtghType::SKIP_TILE_GRP) {
    atgl = {atgh, AtlasTileGroupDataUnit::decodeFrom(bitstream, vuh, vps, asps, afps, atgh)};
  }

  bitstream.rbspTrailingBits();
  return atgl;
}

void AtlasTileGroupLayerRBSP::encodeTo(ostream &stream, const VpccUnitHeader &vuh,
                                       const VpccParameterSet &vps,
                                       const vector<AtlasSequenceParameterSetRBSP> &asps,
                                       const vector<AtlasFrameParameterSetRBSP> &afps) const {
  OutputBitstream bitstream{stream};

  const auto &atgh = atlas_tile_group_header();
  atgh.encodeTo(bitstream, asps, afps);

  if (atgh.atgh_type() != AtghType::SKIP_TILE_GRP) {
    atlas_tile_group_data_unit().encodeTo(bitstream, vuh, vps, asps, afps, atgh);
  }

  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream
