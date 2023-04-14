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

#ifndef TMIV_MIVBITSTREAM_ATLASTILELAYERRBSP_H
#define TMIV_MIVBITSTREAM_ATLASTILELAYERRBSP_H

#include "AtlasFrameParameterSetRBSP.h"
#include "AtlasSequenceParameterSetRBSP.h"
#include "V3cParameterSet.h"
#include "V3cUnit.h"
#include "ViewId.h"

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Common.h>

#include <TMIV/Common/Vector.h>

#include <cstdint>
#include <cstdlib>
#include <iosfwd>
#include <optional>

namespace TMIV::MivBitstream {
enum class AthType : uint8_t { P_TILE, I_TILE, SKIP_TILE };

enum class FlexiblePatchOrientation : uint8_t {
  FPO_NULL,
  FPO_SWAP,
  FPO_MROT270 = FPO_SWAP,
  FPO_ROT90,
  FPO_ROT180,
  FPO_ROT270,
  FPO_MIRROR,
  FPO_MROT90,
  FPO_MROT180,
  FPO_INVALID = UINT8_MAX
};

enum class AtduPatchMode : uint8_t {
  I_INTRA,
  I_RAW,
  I_EOM,
  I_END = 14,
  P_SKIP = I_INTRA,
  P_MERGE,
  P_INTER,
  P_INTRA,
  P_RAW,
  P_EOM,
  P_END = I_END
};

auto operator<<(std::ostream &stream, AthType x) -> std::ostream &;
auto operator<<(std::ostream &stream, FlexiblePatchOrientation x) -> std::ostream &;
auto printTo(std::ostream &stream, AtduPatchMode x, AthType ath_type) -> std::ostream &;

// 23090-5: atlas_tile_header( )
//
// 23090-12 restrictions:
//   * asps_long_term_ref_atlas_frames_flag == 0
//   * afps_raw_3d_offset_bit_count_explicit_mode_flag == 0
//   * ath_type in { I_TILE, SKIP_TILE }
//
// Limitations of the implementation:
//   * asps_num_ref_atlas_frame_lists_in_asps == 1
//   * ath_ref_atlas_frame_list_asps_flag == 1
class AtlasTileHeader {
public:
  [[nodiscard]] constexpr auto ath_no_output_of_prior_atlas_frames_flag() const noexcept;
  [[nodiscard]] constexpr auto ath_atlas_frame_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto ath_atlas_adaptation_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto ath_id() const noexcept;
  [[nodiscard]] constexpr auto ath_type() const noexcept;
  [[nodiscard]] auto ath_atlas_output_flag() const -> bool;
  [[nodiscard]] constexpr auto ath_atlas_frm_order_cnt_lsb() const noexcept;
  [[nodiscard]] constexpr auto ath_ref_atlas_frame_list_asps_flag() const noexcept;
  [[nodiscard]] constexpr auto ath_pos_min_d_quantizer() const noexcept;
  [[nodiscard]] constexpr auto ath_pos_delta_max_d_quantizer() const noexcept;
  [[nodiscard]] auto ath_patch_size_x_info_quantizer() const -> uint8_t;
  [[nodiscard]] auto ath_patch_size_y_info_quantizer() const -> uint8_t;

  constexpr auto ath_no_output_of_prior_atlas_frames_flag(bool value) noexcept -> auto &;
  constexpr auto ath_atlas_frame_parameter_set_id(uint8_t value) -> auto &;
  constexpr auto ath_atlas_adaptation_parameter_set_id(uint8_t value) noexcept -> auto &;
  constexpr auto ath_id(uint8_t value) noexcept -> auto &;
  constexpr auto ath_type(AthType value) noexcept -> auto &;
  constexpr auto ath_atlas_output_flag(bool value) noexcept -> auto &;
  constexpr auto ath_pos_min_d_quantizer(uint8_t value) noexcept -> auto &;
  constexpr auto ath_pos_delta_max_d_quantizer(uint8_t value) noexcept -> auto &;
  constexpr auto ath_atlas_frm_order_cnt_lsb(uint16_t value) noexcept -> auto &;
  constexpr auto ath_ref_atlas_frame_list_asps_flag(bool value) noexcept -> auto &;
  auto ath_patch_size_x_info_quantizer(uint8_t value) noexcept -> AtlasTileHeader &;
  auto ath_patch_size_y_info_quantizer(uint8_t value) noexcept -> AtlasTileHeader &;

  friend auto operator<<(std::ostream &stream, const AtlasTileHeader &x) -> std::ostream &;

  constexpr auto operator==(const AtlasTileHeader &other) const noexcept;
  constexpr auto operator!=(const AtlasTileHeader &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream, const NalUnitHeader &nuh,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV) -> AtlasTileHeader;

  void encodeTo(Common::OutputBitstream &bitstream, const NalUnitHeader &nuh,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV) const;

private:
  std::optional<bool> m_ath_no_output_of_prior_atlas_frames_flag{};
  uint8_t m_ath_atlas_frame_parameter_set_id{};
  uint8_t m_ath_adaptation_parameter_set_id{};
  uint8_t m_ath_id{};
  AthType m_ath_type{};
  std::optional<bool> m_ath_atlas_output_flag{};
  uint16_t m_ath_atlas_frm_order_cnt_lsb{};
  std::optional<bool> m_ath_ref_atlas_frame_list_asps_flag{};
  std::optional<uint8_t> m_ath_pos_min_d_quantizer{};
  std::optional<uint8_t> m_ath_pos_delta_max_d_quantizer{};
  std::optional<uint8_t> m_ath_patch_size_x_info_quantizer{};
  std::optional<uint8_t> m_ath_patch_size_y_info_quantizer{};
};

// 23090-5: skip_patch_data_unit( )
class SkipPatchDataUnit {
public:
  friend auto operator<<(std::ostream &stream, const SkipPatchDataUnit &x) -> std::ostream &;

  constexpr auto operator==(const SkipPatchDataUnit &other) const noexcept;
  constexpr auto operator!=(const SkipPatchDataUnit &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> SkipPatchDataUnit;

  void encodeTo(Common::OutputBitstream &bitstream) const;
};

// 23090-12: pdu_miv_extension( patchIdx )
class PduMivExtension {
public:
  [[nodiscard]] constexpr auto pdu_entity_id() const noexcept;
  [[nodiscard]] auto pdu_depth_occ_threshold() const -> Common::SampleValue;
  [[nodiscard]] auto pdu_texture_offset(uint8_t c) const -> Common::SampleValue;
  [[nodiscard]] constexpr auto pdu_inpaint_flag() const noexcept;

  constexpr auto pdu_entity_id(Common::SampleValue value) noexcept -> auto &;
  constexpr auto pdu_depth_occ_threshold(Common::SampleValue value) noexcept -> auto &;
  auto pdu_texture_offset(uint8_t c, Common::SampleValue value) noexcept -> auto &;
  constexpr auto pdu_inpaint_flag(bool value) noexcept -> auto &;

  auto printTo(std::ostream &stream, uint32_t tileId, size_t patchIdx) const -> std::ostream &;

  auto operator==(const PduMivExtension &other) const -> bool;
  auto operator!=(const PduMivExtension &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const AtlasSequenceParameterSetRBSP &asps) -> PduMivExtension;

  void encodeTo(Common::OutputBitstream &bitstream,
                const AtlasSequenceParameterSetRBSP &asps) const;

private:
  std::optional<Common::SampleValue> m_pdu_entity_id;
  std::optional<Common::SampleValue> m_pdu_depth_occ_threshold;
  std::optional<Common::Vec3sv> m_pdu_texture_offset;
  std::optional<bool> m_pdu_inpaint_flag;
};

// 23090-5: patch_data_unit( patchIdx )
class PatchDataUnit {
public:
  [[nodiscard]] constexpr auto pdu_2d_pos_x() const noexcept;
  [[nodiscard]] constexpr auto pdu_2d_pos_y() const noexcept;
  [[nodiscard]] constexpr auto pdu_2d_size_x_minus1() const noexcept;
  [[nodiscard]] constexpr auto pdu_2d_size_y_minus1() const noexcept;
  [[nodiscard]] constexpr auto pdu_3d_offset_u() const noexcept;
  [[nodiscard]] constexpr auto pdu_3d_offset_v() const noexcept;
  [[nodiscard]] constexpr auto pdu_3d_offset_d() const noexcept;
  [[nodiscard]] auto pdu_3d_range_d() const -> Common::SampleValue;
  [[nodiscard]] constexpr auto pdu_projection_id() const noexcept;
  [[nodiscard]] constexpr auto pdu_orientation_index() const noexcept;
  [[nodiscard]] constexpr auto pdu_lod_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto pdu_lod_scale_x_minus1() const noexcept;
  [[nodiscard]] constexpr auto pdu_lod_scale_y_idc() const noexcept;
  [[nodiscard]] auto pdu_miv_extension() const noexcept -> PduMivExtension;

  constexpr auto pdu_2d_pos_x(uint32_t value) noexcept -> auto &;
  constexpr auto pdu_2d_pos_y(uint32_t value) noexcept -> auto &;
  constexpr auto pdu_2d_size_x_minus1(uint32_t value) noexcept -> auto &;
  constexpr auto pdu_2d_size_y_minus1(uint32_t value) noexcept -> auto &;
  constexpr auto pdu_3d_offset_u(uint32_t value) noexcept -> auto &;
  constexpr auto pdu_3d_offset_v(uint32_t value) noexcept -> auto &;
  constexpr auto pdu_3d_offset_d(Common::SampleValue value) noexcept -> auto &;
  constexpr auto pdu_3d_range_d(Common::SampleValue value) noexcept -> auto &;
  constexpr auto pdu_projection_id(ViewId value) noexcept -> auto &;
  constexpr auto pdu_orientation_index(FlexiblePatchOrientation value) noexcept -> auto &;
  constexpr auto pdu_lod_enabled_flag(bool value) noexcept -> auto &;
  constexpr auto pdu_lod_scale_x_minus1(uint32_t value) noexcept -> auto &;
  constexpr auto pdu_lod_scale_y_idc(uint32_t value) noexcept -> auto &;
  auto pdu_miv_extension(const PduMivExtension &value) noexcept -> PatchDataUnit &;

  [[nodiscard]] constexpr auto pdu_miv_extension() noexcept -> auto &;

  auto printTo(std::ostream &stream, uint32_t tileId, size_t patchIdx) const -> std::ostream &;

  constexpr auto operator==(const PatchDataUnit &other) const noexcept;
  constexpr auto operator!=(const PatchDataUnit &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                         const AtlasTileHeader &ath) -> PatchDataUnit;

  void encodeTo(Common::OutputBitstream &bitstream,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                const AtlasTileHeader &ath) const;

private:
  uint32_t m_pdu_2d_pos_x{};
  uint32_t m_pdu_2d_pos_y{};
  uint32_t m_pdu_2d_size_x_minus1{};
  uint32_t m_pdu_2d_size_y_minus1{};
  uint32_t m_pdu_3d_offset_u{};
  uint32_t m_pdu_3d_offset_v{};
  Common::SampleValue m_pdu_3d_offset_d{};
  std::optional<Common::SampleValue> m_pdu_3d_range_d{};
  ViewId m_pdu_projection_id{};
  FlexiblePatchOrientation m_pdu_orientation_index{};
  std::optional<bool> m_pdu_lod_enabled_flag{};
  std::optional<uint32_t> m_pdu_lod_scale_x_minus1{};
  std::optional<uint32_t> m_pdu_lod_scale_y_idc{};
  std::optional<PduMivExtension> m_pdu_miv_extension;
};

// 23090-5: inter_patch_data_unit( patchIdx )
class InterPatchDataUnit {
public:
  [[nodiscard]] auto ipdu_ref_index() const -> int32_t;
  [[nodiscard]] auto ipdu_patch_index() const -> int32_t;
  [[nodiscard]] auto ipdu_2d_pos_x() const -> int32_t;
  [[nodiscard]] auto ipdu_2d_pos_y() const -> int32_t;
  [[nodiscard]] auto ipdu_2d_delta_size_x() const -> int32_t;
  [[nodiscard]] auto ipdu_2d_delta_size_y() const -> int32_t;
  [[nodiscard]] auto ipdu_3d_offset_u() const -> int32_t;
  [[nodiscard]] auto ipdu_3d_offset_v() const -> int32_t;
  [[nodiscard]] auto ipdu_3d_offset_d() const -> int32_t;
  [[nodiscard]] auto ipdu_3d_range_d() const -> int32_t;
  // [[nodiscard]] auto plr_data() const -> const PlrData &;

  auto ipdu_ref_index(int32_t value) -> InterPatchDataUnit &;
  auto ipdu_patch_index(int32_t value) -> InterPatchDataUnit &;
  auto ipdu_2d_pos_x(int32_t value) -> InterPatchDataUnit &;
  auto ipdu_2d_pos_y(int32_t value) -> InterPatchDataUnit &;
  auto ipdu_2d_delta_size_x(int32_t value) -> InterPatchDataUnit &;
  auto ipdu_2d_delta_size_y(int32_t value) -> InterPatchDataUnit &;
  auto ipdu_3d_offset_u(int32_t value) -> InterPatchDataUnit &;
  auto ipdu_3d_offset_v(int32_t value) -> InterPatchDataUnit &;
  auto ipdu_3d_offset_d(int32_t value) -> InterPatchDataUnit &;
  auto ipdu_3d_range_d(int32_t value) -> InterPatchDataUnit &;
  // [[nodiscard]] auto plr_data() -> PlrData &;

  auto printTo(std::ostream &stream, uint32_t tileId, size_t patchIdx) const -> std::ostream &;

  auto operator==(const InterPatchDataUnit &other) const -> bool;
  auto operator!=(const InterPatchDataUnit &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                         const AtlasTileHeader &ath) -> InterPatchDataUnit;

  void encodeTo(Common::OutputBitstream &bitstream,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                const AtlasTileHeader &ath) const;

private:
  std::optional<int32_t> m_ipdu_ref_index;
  int32_t m_ipdu_patch_index{};
  int32_t m_ipdu_2d_pos_x{};
  int32_t m_ipdu_2d_pos_y{};
  int32_t m_ipdu_2d_delta_size_x{};
  int32_t m_ipdu_2d_delta_size_y{};
  int32_t m_ipdu_3d_offset_u{};
  int32_t m_ipdu_3d_offset_v{};
  int32_t m_ipdu_3d_offset_d{};
  std::optional<int32_t> m_ipdu_3d_range_d;
  // std::optional<PlrData> m_plr_data;
};

// 23090-5: patch_information_data( )
class PatchInformationData {
public:
  [[nodiscard]] auto skip_patch_data_unit() const -> const SkipPatchDataUnit &;
  [[nodiscard]] auto patch_data_unit() const -> const PatchDataUnit &;
  [[nodiscard]] auto inter_patch_data_unit() const -> const InterPatchDataUnit &;

  [[nodiscard]] auto skip_patch_data_unit() -> SkipPatchDataUnit &;
  [[nodiscard]] auto patch_data_unit() -> PatchDataUnit &;
  [[nodiscard]] auto inter_patch_data_unit() -> InterPatchDataUnit &;

  auto printTo(std::ostream &stream, uint32_t tileId, size_t patchIdx) const -> std::ostream &;

  auto operator==(const PatchInformationData &other) const noexcept -> bool;
  auto operator!=(const PatchInformationData &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                         const AtlasTileHeader &ath, AtduPatchMode patchMode)
      -> PatchInformationData;

  void encodeTo(Common::OutputBitstream &bitstream,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV, const AtlasTileHeader &ath,
                AtduPatchMode patchMode) const;

private:
  std::optional<SkipPatchDataUnit> m_skip_patch_data_unit;
  std::optional<PatchDataUnit> m_patch_data_unit;
  std::optional<InterPatchDataUnit> m_inter_patch_data_unit;
};

// 23090-5: atlas_tile_data_unit( )
class AtlasTileDataUnit {
public:
  [[nodiscard]] auto skip_patch_data_unit() const -> const SkipPatchDataUnit &;
  [[nodiscard]] auto atdu_patch_mode(size_t p) const -> AtduPatchMode;
  [[nodiscard]] auto patch_information_data(size_t p) const -> const PatchInformationData &;

  [[nodiscard]] auto skip_patch_data_unit() -> SkipPatchDataUnit &;
  auto atdu_patch_mode(size_t p, AtduPatchMode value) -> AtlasTileDataUnit &;
  [[nodiscard]] auto patch_information_data(size_t p) -> PatchInformationData &;

  [[nodiscard]] auto atduTotalNumberOfPatches() const -> size_t;

  auto printTo(std::ostream &stream, const AtlasTileHeader &ath) const -> std::ostream &;

  auto operator==(const AtlasTileDataUnit &other) const -> bool;
  auto operator!=(const AtlasTileDataUnit &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                         const AtlasTileHeader &ath) -> AtlasTileDataUnit;

  void encodeTo(Common::OutputBitstream &bitstream,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                const AtlasTileHeader &ath) const;

private:
  [[nodiscard]] static auto isEnd(AthType athType, AtduPatchMode patchMode) -> bool;

  std::optional<SkipPatchDataUnit> m_skip_patch_data_unit;
  std::vector<AtduPatchMode> m_atdu_patch_mode;
  std::vector<PatchInformationData> m_patch_information_data;
};

// 23090-5: atlas_tile_layer_rbsp( )
class AtlasTileLayerRBSP {
public:
  [[nodiscard]] constexpr auto atlas_tile_header() const noexcept -> auto &;
  [[nodiscard]] constexpr auto atlas_tile_data_unit() const noexcept -> auto &;

  [[nodiscard]] constexpr auto atlas_tile_header() noexcept -> auto &;
  [[nodiscard]] constexpr auto atlas_tile_data_unit() noexcept -> auto &;

  friend auto operator<<(std::ostream &stream, const AtlasTileLayerRBSP &x) -> std::ostream &;

  auto operator==(const AtlasTileLayerRBSP &other) const noexcept -> bool;
  auto operator!=(const AtlasTileLayerRBSP &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, const NalUnitHeader &nuh,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV)
      -> AtlasTileLayerRBSP;

  void encodeTo(std::ostream &stream, const NalUnitHeader &nuh,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV) const;

private:
  AtlasTileHeader m_atlas_tile_header;
  AtlasTileDataUnit m_atlas_tile_data_unit;
};

using AtlasTileHeaderList = std::vector<AtlasTileHeader>;
} // namespace TMIV::MivBitstream

#include "AtlasTileLayerRBSP.hpp"

#endif
