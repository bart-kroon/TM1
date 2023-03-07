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

#ifndef TMIV_MIVBITSTREAM_EXTENDEDGEOMETRYASSISTANCE_H
#define TMIV_MIVBITSTREAM_EXTENDEDGEOMETRYASSISTANCE_H

#include "CafMivExtension.h"

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Json.h>

#include <vector>

namespace TMIV::MivBitstream {
class EgaBlock {
public:
  auto operator==(const EgaBlock &other) const noexcept -> bool;
  auto operator!=(const EgaBlock &other) const noexcept -> bool;

  [[nodiscard]] auto bbgf_split_flag() const noexcept -> bool;
  [[nodiscard]] auto bbgf_quad_split_flag() const noexcept -> bool;
  [[nodiscard]] auto bbgf_split_orientation_flag() const noexcept -> bool;
  [[nodiscard]] auto bbgf_split_symmetry_flag() const noexcept -> bool;
  [[nodiscard]] auto bbgf_split_first_block_bigger() const noexcept -> bool;
  [[nodiscard]] auto subblocks() const noexcept -> const std::vector<EgaBlock> &;

  auto bbgf_split_flag(bool value) noexcept -> EgaBlock &;
  auto bbgf_quad_split_flag(bool value) noexcept -> EgaBlock &;
  auto bbgf_split_orientation_flag(bool value) noexcept -> EgaBlock &;
  auto bbgf_split_symmetry_flag(bool value) noexcept -> EgaBlock &;
  auto bbgf_split_first_block_bigger(bool value) noexcept -> EgaBlock &;
  auto subblocks(std::vector<EgaBlock> & /*value*/) -> EgaBlock &;

  auto writeTo(std::ostream &stream, uint32_t bbgf_max_number_of_splits, uint32_t blk_y,
               uint32_t blk_x, uint32_t split_lvl) const -> std::ostream &;
  void readFrom(TMIV::Common::Json const &jin, uint32_t bbgf_max_number_of_splits, uint32_t blk_y,
                uint32_t blk_x, uint32_t split_lvl);
  void encodeTo(Common::OutputBitstream &bitstream, uint32_t bbgf_max_number_of_splits,
                uint32_t blk_y, uint32_t blk_x, uint32_t split_lvl) const;
  void decodeFrom(Common::InputBitstream &bitstream, uint32_t bbgf_max_number_of_splits,
                  uint32_t blk_y, uint32_t blk_x, uint32_t split_lvl);

private:
  bool m_bbgf_split_flag{};
  bool m_bbgf_quad_split_flag{};
  bool m_bbgf_split_orientation_flag{};
  bool m_bbgf_split_symmetry_flag{};
  bool m_bbgf_split_first_block_bigger{};

  bool m_bbgf_skip_flag{};
  bool m_bbgf_ltmin_flag{};
  bool m_bbgf_ltmax_flag{};

  int32_t m_bbgf_zmin_delta{};
  int32_t m_bbgf_zmax_delta{};

  std::vector<EgaBlock> m_sub_blocks; // 2 or 4 subblocks.
};

class EgaBlockBasedGeometryFeatures {
public:
  friend auto operator<<(std::ostream &stream, const EgaBlockBasedGeometryFeatures &x)
      -> std::ostream &;

  auto operator==(const EgaBlockBasedGeometryFeatures &other) const noexcept -> bool;
  auto operator!=(const EgaBlockBasedGeometryFeatures &other) const noexcept -> bool;

  [[nodiscard]] auto bbgf_qs() const noexcept -> uint32_t;
  [[nodiscard]] auto bbgf_bw() const noexcept -> uint32_t;
  [[nodiscard]] auto bbgf_max_number_of_splits() const noexcept -> uint8_t;
  [[nodiscard]] auto bbgf_projection_plane_height_minus1() const noexcept -> uint16_t;
  [[nodiscard]] auto bbgf_projection_plane_width_minus1() const noexcept -> uint16_t;
  [[nodiscard]] auto initial_block_grid() const noexcept
      -> const std::vector<std::vector<EgaBlock>> &;

  auto bbgf_qs(uint32_t value) noexcept -> EgaBlockBasedGeometryFeatures &;
  auto bbgf_bw(uint32_t value) noexcept -> EgaBlockBasedGeometryFeatures &;
  auto bbgf_log2_bw_minus2(uint8_t value) noexcept -> EgaBlockBasedGeometryFeatures &;
  auto bbgf_max_number_of_splits(uint8_t value) noexcept -> EgaBlockBasedGeometryFeatures &;

  auto writeTo(std::ostream &stream, uint32_t v_idx) const -> std::ostream &;
  void readFrom(Common::Json const &jin);
  void encodeTo(Common::OutputBitstream &bitstream) const;
  void decodeFrom(Common::InputBitstream &bitstream);

private:
  uint32_t m_bbgf_qs{};
  uint8_t m_bbgf_log2_bw_minus2{};
  uint8_t m_bbgf_max_number_of_splits{};
  uint16_t m_bbgf_projection_plane_height_minus1{};
  uint16_t m_bbgf_projection_plane_width_minus1{};
  std::vector<std::vector<EgaBlock>> m_bbgf_initial_block_grid; // index: y, x.
};

class ExtendedGeometryAssistance {
public:
  friend auto operator<<(std::ostream &stream, const ExtendedGeometryAssistance &x)
      -> std::ostream &;

  auto operator==(const ExtendedGeometryAssistance &other) const noexcept -> bool;
  auto operator!=(const ExtendedGeometryAssistance &other) const noexcept -> bool;

  [[nodiscard]] auto ega_num_views() const noexcept -> uint16_t;
  [[nodiscard]] auto ega_num_available_assistance_types_minus1() const noexcept -> uint8_t;
  [[nodiscard]] auto ega_assistance_present_flag() const noexcept -> const std::vector<bool> &;
  [[nodiscard]] auto ega_assistance_type_present_flag() const noexcept
      -> const std::vector<std::vector<bool>> &;
  [[nodiscard]] auto block_based_geometry_features() const noexcept
      -> const std::vector<EgaBlockBasedGeometryFeatures> &;

  auto ega_num_views_minus1(uint16_t value) noexcept -> ExtendedGeometryAssistance &;
  auto ega_num_available_assistance_types_minus1(uint8_t value) noexcept
      -> ExtendedGeometryAssistance &;

  void writeTo(std::ostream &stream) const;
  static auto readFrom(Common::Json const &jin) -> ExtendedGeometryAssistance;
  void encodeTo(Common::OutputBitstream &bitstream) const;
  static auto decodeFrom(Common::InputBitstream &bitstream) -> ExtendedGeometryAssistance;

private:
  uint16_t m_ega_num_views_minus1{};
  uint8_t m_ega_num_available_assistance_types_minus1{};
  std::vector<bool> m_ega_assistance_present_flag;
  std::vector<std::vector<bool>> m_ega_assistance_type_present_flag;
  std::vector<EgaBlockBasedGeometryFeatures> m_ega_block_based_geometry_features; // index: view.
};
} // namespace TMIV::MivBitstream

#endif