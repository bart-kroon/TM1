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

#ifndef _TMIV_MIVBITSTREAM_ATLASTILEGROUPLAYERRBSP_H_
#define _TMIV_MIVBITSTREAM_ATLASTILEGROUPLAYERRBSP_H_

#include <TMIV/Common/Bitstream.h>
#include <TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/VpccParameterSet.h>
#include <TMIV/MivBitstream/VpccUnit.h>

#include <cstdint>
#include <cstdlib>
#include <iosfwd>
#include <optional>
#include <variant>

namespace TMIV::MivBitstream {
enum class AtghType : std::uint8_t { P_TILE_GRP, I_TILE_GRP, SKIP_TILE_GRP };

enum class FlexiblePatchOrientation : std::uint8_t {
  FPO_NULL,
  FPO_SWAP,
  FPO_ROT90,
  FPO_ROT180,
  FPO_ROT270,
  FPO_MIRROR,
  FPO_MROT90,
  FPO_MROT180,
  FPO_INVALID = UINT8_MAX
};

enum class AtgduPatchMode : std::uint8_t {
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

auto operator<<(std::ostream &stream, AtghType x) -> std::ostream &;
auto operator<<(std::ostream &stream, FlexiblePatchOrientation x) -> std::ostream &;
auto printTo(std::ostream &stream, AtgduPatchMode x, AtghType atgh_type) -> std::ostream &;

// 23090-5: atlas_tile_group_header()
class AtlasTileGroupHeader {
public:
  [[nodiscard]] constexpr auto atgh_atlas_frame_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto atgh_adaptation_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto atgh_address() const noexcept;
  [[nodiscard]] constexpr auto atgh_type() const noexcept;
  [[nodiscard]] auto atgh_atlas_output_flag() const noexcept -> bool;
  [[nodiscard]] constexpr auto atgh_atlas_frm_order_cnt_lsb() const noexcept;
  [[nodiscard]] auto atgh_patch_size_x_info_quantizer() const noexcept -> std::uint8_t;
  [[nodiscard]] auto atgh_patch_size_y_info_quantizer() const noexcept -> std::uint8_t;
  [[nodiscard]] constexpr auto atgh_pos_min_z_quantizer() const noexcept { return std::uint8_t(0); }
  [[nodiscard]] constexpr auto atgh_pos_max_z_quantizer() const noexcept { return std::uint8_t(0); }

  constexpr auto atgh_atlas_frame_parameter_set_id(const std::uint8_t value) noexcept -> auto &;
  constexpr auto atgh_adaptation_parameter_set_id(const std::uint8_t value) noexcept -> auto &;
  constexpr auto atgh_address(const std::uint8_t value) noexcept -> auto &;
  constexpr auto atgh_type(const AtghType value) noexcept -> auto &;
  constexpr auto atgh_atlas_output_flag(const bool value) noexcept -> auto &;
  constexpr auto atgh_atlas_frm_order_cnt_lsb(const std::uint8_t value) noexcept -> auto &;
  auto atgh_patch_size_x_info_quantizer(const std::uint8_t value) noexcept
      -> AtlasTileGroupHeader &;
  auto atgh_patch_size_y_info_quantizer(const std::uint8_t value) noexcept
      -> AtlasTileGroupHeader &;

  friend auto operator<<(std::ostream &stream, const AtlasTileGroupHeader &x) -> std::ostream &;

  constexpr auto operator==(const AtlasTileGroupHeader &other) const noexcept;
  constexpr auto operator!=(const AtlasTileGroupHeader &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV)
      -> AtlasTileGroupHeader;

  void encodeTo(Common::OutputBitstream &bitstream,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV) const;

private:
  std::uint8_t m_atgh_atlas_frame_parameter_set_id{};
  std::uint8_t m_atgh_adaptation_parameter_set_id{};
  std::uint8_t m_atgh_address{};
  AtghType m_atgh_type{};
  std::optional<bool> m_atgh_atlas_output_flag{};
  std::uint8_t m_atgh_atlas_frm_order_cnt_lsb{};
  std::uint8_t m_atgh_patch_size_x_info_quantizer{};
  std::uint8_t m_atgh_patch_size_y_info_quantizer{};
};

// 23090-5: skip_patch_data_unit(patchIdx)
class SkipPatchDataUnit {
public:
  friend auto operator<<(std::ostream &stream, const SkipPatchDataUnit &x) -> std::ostream &;

  constexpr auto operator==(const SkipPatchDataUnit &other) const noexcept;
  constexpr auto operator!=(const SkipPatchDataUnit &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> SkipPatchDataUnit;

  void encodeTo(Common::OutputBitstream &bitstream) const;
};

// 23090-12: patch_data_unit(patchIdx)
class PatchDataUnit {
public:
  [[nodiscard]] constexpr auto pdu_2d_pos_x() const noexcept;
  [[nodiscard]] constexpr auto pdu_2d_pos_y() const noexcept;
  [[nodiscard]] constexpr auto pdu_2d_size_x_minus1() const noexcept;
  [[nodiscard]] constexpr auto pdu_2d_size_y_minus1() const noexcept;
  [[nodiscard]] constexpr auto pdu_view_pos_x() const noexcept;
  [[nodiscard]] constexpr auto pdu_view_pos_y() const noexcept;
  [[nodiscard]] constexpr auto pdu_depth_start() const noexcept;
  [[nodiscard]] auto pdu_depth_end() const noexcept -> std::uint32_t;
  [[nodiscard]] constexpr auto pdu_view_id() const noexcept;
  [[nodiscard]] constexpr auto pdu_orientation_index() const noexcept;
  [[nodiscard]] constexpr auto pdu_entity_id() const noexcept;
  [[nodiscard]] auto pdu_depth_occ_map_threshold() const noexcept -> std::uint32_t;

  constexpr auto pdu_2d_pos_x(const std::uint16_t value) noexcept -> auto &;
  constexpr auto pdu_2d_pos_y(const std::uint16_t value) noexcept -> auto &;
  constexpr auto pdu_2d_size_x_minus1(const std::uint16_t value) noexcept -> auto &;
  constexpr auto pdu_2d_size_y_minus1(const std::uint16_t value) noexcept -> auto &;
  constexpr auto pdu_view_pos_x(const std::uint16_t value) noexcept -> auto &;
  constexpr auto pdu_view_pos_y(const std::uint16_t value) noexcept -> auto &;
  constexpr auto pdu_depth_start(const std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_depth_end(const std::uint32_t value) noexcept -> auto &;
  constexpr auto pdu_view_id(const std::uint16_t value) noexcept -> auto &;
  constexpr auto pdu_orientation_index(const FlexiblePatchOrientation value) noexcept -> auto &;
  constexpr auto pdu_entity_id(const unsigned value) noexcept -> auto &;
  constexpr auto pdu_depth_occ_map_threshold(const std::uint32_t value) noexcept -> auto &;

  auto printTo(std::ostream &stream, std::size_t patchIdx) const -> std::ostream &;

  constexpr auto operator==(const PatchDataUnit &other) const noexcept;
  constexpr auto operator!=(const PatchDataUnit &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream, const VpccUnitHeader &vuh,
                         const VpccParameterSet &vps,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsVector,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsVector,
                         const AtlasTileGroupHeader &atgh) -> PatchDataUnit;

  void encodeTo(Common::OutputBitstream &bitstream, const VpccUnitHeader &vuh,
                const VpccParameterSet &vps,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsVector,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsVector,
                const AtlasTileGroupHeader &atgh) const;

private:
  std::uint16_t m_pdu_2d_pos_x{};
  std::uint16_t m_pdu_2d_pos_y{};
  std::uint16_t m_pdu_2d_size_x_minus1{};
  std::uint16_t m_pdu_2d_size_y_minus1{};
  std::uint16_t m_pdu_view_pos_x{};
  std::uint16_t m_pdu_view_pos_y{};
  std::uint32_t m_pdu_depth_start{};
  std::optional<std::uint32_t> m_pdu_depth_end;
  std::uint16_t m_pdu_view_id{};
  FlexiblePatchOrientation m_pdu_orientation_index{};
  unsigned m_pdu_entity_id{};
  std::optional<std::uint32_t> m_pdu_depth_occ_map_threshold;
};

// 23090-5: patch_information_data()
class PatchInformationData {
public:
  using Data = std::variant<std::monostate, SkipPatchDataUnit, PatchDataUnit>;

  PatchInformationData() = default;

  template <typename Value>
  constexpr explicit PatchInformationData(Value &&value) : m_data{std::forward<Value>(value)} {}

            [[nodiscard]] constexpr auto data() const noexcept -> auto &;

  [[nodiscard]] auto skip_patch_data_unit() const noexcept -> const SkipPatchDataUnit &;
  [[nodiscard]] auto patch_data_unit() const noexcept -> const PatchDataUnit &;

  auto printTo(std::ostream &stream, std::size_t patchIdx) const -> std::ostream &;

  auto operator==(const PatchInformationData &other) const noexcept -> bool;
  auto operator!=(const PatchInformationData &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const VpccUnitHeader &vuh,
                         const VpccParameterSet &vps,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                         const AtlasTileGroupHeader &atgh, AtgduPatchMode patchMode)
      -> PatchInformationData;

  void encodeTo(Common::OutputBitstream &bitstream, const VpccUnitHeader &vuh,
                const VpccParameterSet &vps,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                const AtlasTileGroupHeader &atgh, AtgduPatchMode patchMode) const;

private:
  Data m_data;
};

// 23090-5: atlas_tile_group_data_unit()
class AtlasTileGroupDataUnit {
public:
  using Vector = std::vector<std::pair<AtgduPatchMode, PatchInformationData>>;

  AtlasTileGroupDataUnit() = default;

  template <typename... Args>
  explicit AtlasTileGroupDataUnit(Args &&... args) : m_vector{std::forward<Args>(args)...} {}

            [[nodiscard]] auto atgduTotalNumberOfPatches() const noexcept -> std::size_t;
  [[nodiscard]] auto atgdu_patch_mode(std::size_t p) const -> AtgduPatchMode;
  [[nodiscard]] auto patch_information_data(std::size_t p) const -> const PatchInformationData &;

  // Visit all elements in the atlas tile group data unit in ascending order. The expected signature
  // of the visitor is: void(std::size_t p, AtgduPatchMode, const PatchInformationData &)
  template <typename Visitor> void visit(Visitor &&visitor) const;

  auto printTo(std::ostream &stream, AtghType atgh_type) const -> std::ostream &;

  auto operator==(const AtlasTileGroupDataUnit &other) const -> bool;
  auto operator!=(const AtlasTileGroupDataUnit &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const VpccUnitHeader &vuh,
                         const VpccParameterSet &vps,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                         const AtlasTileGroupHeader &atgh) -> AtlasTileGroupDataUnit;

  void encodeTo(Common::OutputBitstream &bitstream, const VpccUnitHeader &vuh,
                const VpccParameterSet &vps,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV,
                const AtlasTileGroupHeader &atgh) const;

private:
  Vector m_vector;
};

// 23090-5: atlas_tile_group_layer_rbsp()
class AtlasTileGroupLayerRBSP {
public:
  AtlasTileGroupLayerRBSP() = default;
  explicit AtlasTileGroupLayerRBSP(AtlasTileGroupHeader header)
      : m_atlas_tile_group_header{header} {}

  AtlasTileGroupLayerRBSP(AtlasTileGroupHeader header, AtlasTileGroupDataUnit unit)
      : m_atlas_tile_group_header{header}, m_atlas_tile_group_data_unit{std::move(unit)} {}

  template <typename... AtgduArgs>
  AtlasTileGroupLayerRBSP(AtlasTileGroupHeader header, std::in_place_t in_place,
                          AtgduArgs &&... args)
      : m_atlas_tile_group_header{header}, m_atlas_tile_group_data_unit{
                                               in_place, std::forward<AtgduArgs>(args)...} {}

            [[nodiscard]] constexpr auto atlas_tile_group_header() const noexcept
        -> const AtlasTileGroupHeader &;
  [[nodiscard]] auto atlas_tile_group_data_unit() const noexcept -> const AtlasTileGroupDataUnit &;

  friend auto operator<<(std::ostream &stream, const AtlasTileGroupLayerRBSP &x) -> std::ostream &;

  auto operator==(const AtlasTileGroupLayerRBSP &other) const noexcept -> bool;
  auto operator!=(const AtlasTileGroupLayerRBSP &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, const VpccUnitHeader &vuh,
                         const VpccParameterSet &vps,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                         const std::vector<AtlasFrameParameterSetRBSP> &afpsV)
      -> AtlasTileGroupLayerRBSP;

  void encodeTo(std::ostream &stream, const VpccUnitHeader &vuh, const VpccParameterSet &vps,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV,
                const std::vector<AtlasFrameParameterSetRBSP> &afpsV) const;

private:
  AtlasTileGroupHeader m_atlas_tile_group_header;
  std::optional<AtlasTileGroupDataUnit> m_atlas_tile_group_data_unit;
};
} // namespace TMIV::MivBitstream

#include "AtlasTileGroupLayerRBSP.hpp"

#endif
