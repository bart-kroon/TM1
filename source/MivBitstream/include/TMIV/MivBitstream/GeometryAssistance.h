/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#ifndef TMIV_MIVBITSTREAM_GEOMETRYASSISTANCE_H
#define TMIV_MIVBITSTREAM_GEOMETRYASSISTANCE_H

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Json.h>
#include <TMIV/MivBitstream/CafMivExtension.h>

#include <vector>

namespace TMIV::MivBitstream {
// // 23090-12: Geometry assistance

class GaSubBlock {
public:
  auto operator==(const GaSubBlock &other) const noexcept -> bool;
  auto operator!=(const GaSubBlock &other) const noexcept -> bool;

  [[nodiscard]] auto gas_skip_flag() const noexcept -> bool;
  [[nodiscard]] auto gas_ltmin_flag() const noexcept -> bool;
  [[nodiscard]] auto gas_ltmax_flag() const noexcept -> bool;
  [[nodiscard]] auto gas_zmin_delta() const noexcept -> int32_t;
  [[nodiscard]] auto gas_zmax_delta() const noexcept -> int32_t;

  auto gas_skip_flag(bool value) noexcept -> GaSubBlock &;
  auto gas_ltmin_flag(bool value) noexcept -> GaSubBlock &;
  auto gas_ltmax_flag(bool value) noexcept -> GaSubBlock &;
  auto gas_zmin_delta(int32_t value) noexcept -> GaSubBlock &;
  auto gas_zmax_delta(int32_t value) noexcept -> GaSubBlock &;

  auto writeTo(std::ostream &stream, unsigned int blk_y, unsigned int blk_x, unsigned int sb_y,
               unsigned int sb_x) const -> std::ostream &;
  void readFrom(TMIV::Common::Json const &jin, unsigned int blk_y, unsigned int blk_x,
                unsigned int sb_y, unsigned int sb_x);
  void encodeTo(Common::OutputBitstream &bitstream, unsigned int blk_y, unsigned int blk_x,
                unsigned int sb_y, unsigned int sb_x) const;
  void decodeFrom(Common::InputBitstream &bitstream, unsigned int blk_y, unsigned int blk_x,
                  unsigned int sb_y, unsigned int sb_x);

private:
  bool m_gas_skip_flag{};
  bool m_gas_ltmin_flag{};
  bool m_gas_ltmax_flag{};
  int32_t m_gas_zmin_delta{};
  int32_t m_gas_zmax_delta{};
};

class GaBlock {
public:
  auto operator==(const GaBlock &other) const noexcept -> bool;
  auto operator!=(const GaBlock &other) const noexcept -> bool;

  [[nodiscard]] auto gas_split_flag() const noexcept -> bool;
  [[nodiscard]] auto gas_quad_split_flag() const noexcept -> bool;
  [[nodiscard]] auto gas_split_orientation_flag() const noexcept -> bool;
  [[nodiscard]] auto gas_split_symmetry_flag() const noexcept -> bool;
  [[nodiscard]] auto gas_split_first_block_bigger() const noexcept -> bool;
  [[nodiscard]] auto subblocks() const noexcept -> const std::vector<GaSubBlock> &;

  auto gas_split_flag(bool value) noexcept -> GaBlock &;
  auto gas_quad_split_flag(bool value) noexcept -> GaBlock &;
  auto gas_split_orientation_flag(bool value) noexcept -> GaBlock &;
  auto gas_split_symmetry_flag(bool value) noexcept -> GaBlock &;
  auto gas_split_first_block_bigger(bool value) noexcept -> GaBlock &;
  auto subblocks(std::vector<GaSubBlock> & /*value*/) -> GaBlock &;

  auto writeTo(std::ostream &stream, unsigned int blk_y, unsigned int blk_x) const
      -> std::ostream &;
  void readFrom(TMIV::Common::Json const &jin, unsigned int blk_y, unsigned int blk_x);
  void encodeTo(Common::OutputBitstream &bitstream, unsigned int blk_y, unsigned int blk_x) const;
  void decodeFrom(Common::InputBitstream &bitstream, unsigned int blk_y, unsigned int blk_x);

private:
  bool m_gas_split_flag{};
  bool m_gas_quad_split_flag{};
  bool m_gas_split_orientation_flag{};
  bool m_gas_split_symmetry_flag{};
  bool m_gas_split_first_block_bigger{};

  std::vector<GaSubBlock> m_sub_blocks; // 1, 2 or 4 subblocks.
};

class GeometryAssistance {
public:
  auto operator==(const GeometryAssistance &other) const noexcept -> bool;
  auto operator!=(const GeometryAssistance &other) const noexcept -> bool;

  [[nodiscard]] auto gas_qs() const noexcept -> uint32_t;
  [[nodiscard]] auto gas_num_views() const noexcept -> uint16_t;
  [[nodiscard]] auto gas_bw() const noexcept -> uint32_t;
  [[nodiscard]] auto gas_projection_plane_height_minus1() const noexcept
      -> const std::vector<uint16_t> &;
  [[nodiscard]] auto gas_projection_plane_width_minus1() const noexcept
      -> const std::vector<uint16_t> &;
  [[nodiscard]] auto blocks() const noexcept
      -> const std::vector<std::vector<std::vector<GaBlock>>> &;

  auto gas_qs(uint32_t value) noexcept -> GeometryAssistance &;
  auto gas_bw(uint32_t value) noexcept -> GeometryAssistance &;
  auto gas_num_views_minus1(uint16_t value) noexcept -> GeometryAssistance &;
  auto gas_log2_bw_minus2(uint8_t value) noexcept -> GeometryAssistance &;
  auto blocks(std::vector<std::vector<std::vector<GaBlock>>> &value) -> GeometryAssistance &;

  void writeTo(std::ostream &stream) const;
  static auto readFrom(Common::Json const &jin) -> GeometryAssistance;
  void encodeTo(Common::OutputBitstream &bitstream) const;
  static auto decodeFrom(Common::InputBitstream &bitstream) -> GeometryAssistance;

private:
  uint32_t m_gas_qs{};
  uint16_t m_gas_num_views_minus1{};
  uint8_t m_gas_log2_bw_minus2{};
  std::vector<uint16_t> m_gas_projection_plane_height_minus1;
  std::vector<uint16_t> m_gas_projection_plane_width_minus1;
  std::vector<std::vector<std::vector<GaBlock>>> m_gas_blocks; // index: view, y, x.
};
} // namespace TMIV::MivBitstream

#endif
