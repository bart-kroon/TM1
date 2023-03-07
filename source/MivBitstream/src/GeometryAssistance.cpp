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

#include <TMIV/Common/Json.h>
#include <TMIV/MivBitstream/GeometryAssistance.h>

namespace TMIV::MivBitstream {
auto GaSubBlock::gas_skip_flag() const noexcept -> bool { return m_gas_skip_flag; }

auto GaSubBlock::gas_ltmin_flag() const noexcept -> bool { return m_gas_ltmin_flag; }

auto GaSubBlock::gas_ltmax_flag() const noexcept -> bool { return m_gas_ltmax_flag; }

auto GaSubBlock::gas_zmin_delta() const noexcept -> int32_t { return m_gas_zmin_delta; }

auto GaSubBlock::gas_zmax_delta() const noexcept -> int32_t { return m_gas_zmax_delta; }

auto GaSubBlock::gas_skip_flag(bool value) noexcept -> GaSubBlock & {
  m_gas_skip_flag = value;
  return *this;
}

auto GaSubBlock::gas_ltmin_flag(bool value) noexcept -> GaSubBlock & {
  m_gas_ltmin_flag = value;
  return *this;
}

auto GaSubBlock::gas_ltmax_flag(bool value) noexcept -> GaSubBlock & {
  m_gas_ltmax_flag = value;
  return *this;
}

auto GaSubBlock::gas_zmin_delta(int32_t value) noexcept -> GaSubBlock & {
  m_gas_zmin_delta = value;
  return *this;
}

auto GaSubBlock::gas_zmax_delta(int32_t value) noexcept -> GaSubBlock & {
  m_gas_zmax_delta = value;
  return *this;
}

auto GaSubBlock::operator==(const GaSubBlock &other) const noexcept -> bool {
  return gas_skip_flag() == other.gas_skip_flag() && gas_ltmin_flag() == other.gas_ltmin_flag() &&
         gas_ltmax_flag() == other.gas_ltmax_flag() && gas_zmin_delta() == other.gas_zmin_delta() &&
         gas_zmax_delta() == other.gas_zmax_delta();
}

auto GaSubBlock::operator!=(const GaSubBlock &other) const noexcept -> bool {
  return !operator==(other);
}

auto GaSubBlock::writeTo(std::ostream &stream, uint32_t blk_y, uint32_t blk_x, uint32_t sb_y,
                         uint32_t sb_x) const -> std::ostream & {
  stream << " [";
  stream << "gas_skip_flag=" << m_gas_skip_flag;
  if (m_gas_skip_flag) {
    stream << "]";
    return stream;
  }
  if ((blk_y == 0 && sb_y == 0) || (blk_x == 0 && sb_x == 0)) {
    ; // no neighbor signaling.
  } else {
    stream << ",gas_ltmin_flag=" << m_gas_ltmin_flag;
    stream << ",gas_ltmax_flag=" << m_gas_ltmax_flag;
  }
  stream << ",gas_zmin_delta=" << m_gas_zmin_delta;
  stream << ",gas_zmax_delta=" << m_gas_zmax_delta;

  stream << "]";
  return stream;
}

void GaSubBlock::readFrom(TMIV::Common::Json const &jin, uint32_t blk_y, uint32_t blk_x,
                          uint32_t sb_y, uint32_t sb_x) {
  m_gas_skip_flag = jin.require("gas_skip_flag").as<int32_t>() != 0;
  if (m_gas_skip_flag) {
    return;
  }
  if ((blk_y == 0 && sb_y == 0) || (blk_x == 0 && sb_x == 0)) {
    ; // no neighbor signaling.
  } else {
    m_gas_ltmin_flag = jin.require("gas_ltmin_flag").as<int32_t>() != 0;
    m_gas_ltmax_flag = jin.require("gas_ltmax_flag").as<int32_t>() != 0;
  }

  m_gas_zmin_delta = jin.require("gas_zmin_delta").as<int32_t>();
  m_gas_zmax_delta = jin.require("gas_zmax_delta").as<int32_t>();
}

void GaSubBlock::encodeTo(Common::OutputBitstream &bitstream, uint32_t blk_y, uint32_t blk_x,
                          uint32_t sb_y, uint32_t sb_x) const {
  bitstream.putFlag(m_gas_skip_flag);
  if (m_gas_skip_flag) {
    return;
  }
  if ((blk_y == 0 && sb_y == 0) || (blk_x == 0 && sb_x == 0)) {
    ; // no neighbor signaling.
  } else {
    bitstream.putFlag(m_gas_ltmin_flag);
    bitstream.putFlag(m_gas_ltmax_flag);
  }
  bitstream.putSExpGolomb(m_gas_zmin_delta);
  bitstream.putSExpGolomb(m_gas_zmax_delta);
}

void GaSubBlock::decodeFrom(Common::InputBitstream &bitstream, uint32_t blk_y, uint32_t blk_x,
                            uint32_t sb_y, uint32_t sb_x) {
  m_gas_skip_flag = bitstream.getFlag();
  if (m_gas_skip_flag) {
    return;
  }
  if ((blk_y == 0 && sb_y == 0) || (blk_x == 0 && sb_x == 0)) {
    ; // no neighbor signaling.
  } else {
    m_gas_ltmin_flag = bitstream.getFlag();
    m_gas_ltmax_flag = bitstream.getFlag();
  }
  m_gas_zmin_delta = bitstream.getSExpGolomb<int32_t>();
  m_gas_zmax_delta = bitstream.getSExpGolomb<int32_t>();
}

auto GaBlock::gas_split_flag() const noexcept -> bool { return m_gas_split_flag; }

auto GaBlock::gas_quad_split_flag() const noexcept -> bool { return m_gas_quad_split_flag; }

auto GaBlock::gas_split_orientation_flag() const noexcept -> bool {
  return m_gas_split_orientation_flag;
}

auto GaBlock::gas_split_symmetry_flag() const noexcept -> bool { return m_gas_split_symmetry_flag; }

auto GaBlock::gas_split_first_block_bigger() const noexcept -> bool {
  return m_gas_split_first_block_bigger;
}

auto GaBlock::subblocks() const noexcept -> const std::vector<GaSubBlock> & { return m_sub_blocks; }

auto GaBlock::gas_split_flag(bool value) noexcept -> GaBlock & {
  m_gas_split_flag = value;
  return *this;
}

auto GaBlock::gas_quad_split_flag(bool value) noexcept -> GaBlock & {
  m_gas_quad_split_flag = value;
  return *this;
}

auto GaBlock::gas_split_orientation_flag(bool value) noexcept -> GaBlock & {
  m_gas_split_orientation_flag = value;
  return *this;
}

auto GaBlock::gas_split_symmetry_flag(bool value) noexcept -> GaBlock & {
  m_gas_split_symmetry_flag = value;
  return *this;
}

auto GaBlock::gas_split_first_block_bigger(bool value) noexcept -> GaBlock & {
  m_gas_split_first_block_bigger = value;
  return *this;
}

auto GaBlock::subblocks(std::vector<GaSubBlock> &value) -> GaBlock & {
  m_sub_blocks = value;
  return *this;
}

auto GaBlock::operator==(const GaBlock &other) const noexcept -> bool {
  return gas_split_flag() == other.gas_split_flag() &&
         gas_quad_split_flag() == other.gas_quad_split_flag() &&
         gas_split_orientation_flag() == other.gas_split_orientation_flag() &&
         gas_split_symmetry_flag() == other.gas_split_symmetry_flag() &&
         gas_split_first_block_bigger() == other.gas_split_first_block_bigger() &&
         subblocks() == other.subblocks();
}

auto GaBlock::operator!=(const GaBlock &other) const noexcept -> bool { return !operator==(other); }

auto GaBlock::writeTo(std::ostream &stream, uint32_t blk_y, uint32_t blk_x) const
    -> std::ostream & {
  stream << " gas_split_flag=" << m_gas_split_flag;
  uint32_t n_sb_x = 0;
  uint32_t n_sb_y = 0;
  if (m_gas_split_flag) {
    stream << " gas_quad_split_flag=" << m_gas_quad_split_flag;
    if (m_gas_quad_split_flag) {
      n_sb_x = n_sb_y = 2;
    } else {
      stream << " gas_split_orientation_flag=" << m_gas_split_orientation_flag;
      stream << " gas_split_symmetry_flag=" << m_gas_split_symmetry_flag;
      if (!m_gas_split_symmetry_flag) {
        stream << " gas_split_first_block_bigger=" << m_gas_split_first_block_bigger;
      }
      if (m_gas_split_orientation_flag) {
        n_sb_x = 2;
        n_sb_y = 1;
      } else {
        n_sb_x = 1;
        n_sb_y = 2;
      }
    }
  } else {
    n_sb_x = n_sb_y = 1;
  }
  uint32_t sb_idx = 0;
  for (uint32_t sb_y = 0; sb_y < n_sb_y; sb_y++) {
    for (uint32_t sb_x = 0; sb_x < n_sb_x; sb_x++, sb_idx++) {
      m_sub_blocks[sb_idx].writeTo(stream, blk_y, blk_x, sb_y, sb_x);
    }
  }
  return stream;
}

void GaBlock::readFrom(TMIV::Common::Json const &jin, uint32_t blk_y, uint32_t blk_x) {
  uint32_t n_sb_x = 1;
  uint32_t n_sb_y = 1;
  m_gas_split_flag = jin.require("gas_split_flag").as<int32_t>() != 0;
  if (m_gas_split_flag) {
    m_gas_quad_split_flag = jin.require("gas_quad_split_flag").as<int32_t>() != 0;
    if (m_gas_quad_split_flag) {
      n_sb_x = n_sb_y = 2;
    } else {
      m_gas_split_orientation_flag = jin.require("gas_split_orientation_flag").as<int32_t>() != 0;
      m_gas_split_symmetry_flag = jin.require("gas_split_symmetry_flag").as<int32_t>() != 0;
      if (!m_gas_split_symmetry_flag) {
        m_gas_split_first_block_bigger =
            jin.require("gas_split_first_block_bigger").as<int32_t>() != 0;
      }
      if (m_gas_split_orientation_flag) {
        n_sb_x = 2;
        n_sb_y = 1;
      } else {
        n_sb_x = 1;
        n_sb_y = 2;
      }
    }
  }
  uint32_t sb_idx = 0;
  auto subblks = jin.require("subblks").as<Common::Json::Array>();
  m_sub_blocks.resize(n_sb_y * n_sb_x);
  for (uint32_t sb_y = 0; sb_y < n_sb_y; sb_y++) {
    for (uint32_t sb_x = 0; sb_x < n_sb_x; sb_x++, sb_idx++) {
      m_sub_blocks[sb_idx].readFrom(subblks.at(sb_idx), blk_y, blk_x, sb_y, sb_x);
    }
  }
}

void GaBlock::encodeTo(Common::OutputBitstream &bitstream, uint32_t blk_y, uint32_t blk_x) const {
  uint32_t n_sb_x = 1;
  uint32_t n_sb_y = 1;
  bitstream.putFlag(m_gas_split_flag);
  if (m_gas_split_flag) {
    bitstream.putFlag(m_gas_quad_split_flag);
    if (m_gas_quad_split_flag) {
      n_sb_x = n_sb_y = 2;
    } else {
      bitstream.putFlag(m_gas_split_orientation_flag);
      bitstream.putFlag(m_gas_split_symmetry_flag);
      if (!m_gas_split_symmetry_flag) {
        bitstream.putFlag(m_gas_split_first_block_bigger);
      }
      if (m_gas_split_orientation_flag) {
        n_sb_x = 2;
      } else {
        n_sb_y = 2;
      }
    }
  }
  uint32_t sb_idx = 0;
  for (uint32_t sb_y = 0; sb_y < n_sb_y; sb_y++) {
    for (uint32_t sb_x = 0; sb_x < n_sb_x; sb_x++, sb_idx++) {
      m_sub_blocks[sb_idx].encodeTo(bitstream, blk_y, blk_x, sb_y, sb_x);
    }
  }
}

void GaBlock::decodeFrom(Common::InputBitstream &bitstream, uint32_t blk_y, uint32_t blk_x) {
  uint32_t n_sb_x = 1;
  uint32_t n_sb_y = 1;
  m_gas_split_flag = bitstream.getFlag();
  if (m_gas_split_flag) {
    m_gas_quad_split_flag = bitstream.getFlag();
    if (m_gas_quad_split_flag) {
      n_sb_x = n_sb_y = 2;
    } else {
      m_gas_split_orientation_flag = bitstream.getFlag();
      m_gas_split_symmetry_flag = bitstream.getFlag();
      if (!m_gas_split_symmetry_flag) {
        m_gas_split_first_block_bigger = bitstream.getFlag();
      }
      if (m_gas_split_orientation_flag) {
        n_sb_x = 2;
      } else {
        n_sb_y = 2;
      }
    }
  }
  uint32_t sb_idx = 0;
  m_sub_blocks.resize(n_sb_y * n_sb_x);
  for (uint32_t sb_y = 0; sb_y < n_sb_y; sb_y++) {
    for (uint32_t sb_x = 0; sb_x < n_sb_x; sb_x++, sb_idx++) {
      m_sub_blocks[sb_idx].decodeFrom(bitstream, blk_y, blk_x, sb_y, sb_x);
    }
  }
}

auto GeometryAssistance::gas_qs() const noexcept -> uint32_t { return m_gas_qs; }

auto GeometryAssistance::gas_bw() const noexcept -> uint32_t {
  uint32_t result = 1 << (m_gas_log2_bw_minus2 + 2);
  return result;
}

auto GeometryAssistance::gas_num_views() const noexcept -> uint16_t {
  return m_gas_num_views_minus1 + 1;
}

auto GeometryAssistance::gas_projection_plane_height_minus1() const noexcept
    -> const std::vector<uint16_t> & {
  return m_gas_projection_plane_height_minus1;
}

auto GeometryAssistance::gas_projection_plane_width_minus1() const noexcept
    -> const std::vector<uint16_t> & {
  return m_gas_projection_plane_width_minus1;
}

auto GeometryAssistance::blocks() const noexcept
    -> const std::vector<std::vector<std::vector<GaBlock>>> & {
  return m_gas_blocks;
}

auto GeometryAssistance::gas_qs(uint32_t value) noexcept -> GeometryAssistance & {
  m_gas_qs = value;
  return *this;
}

auto GeometryAssistance::gas_num_views_minus1(uint16_t value) noexcept -> GeometryAssistance & {
  m_gas_num_views_minus1 = value;
  return *this;
}

auto GeometryAssistance::gas_log2_bw_minus2(uint8_t value) noexcept -> GeometryAssistance & {
  m_gas_log2_bw_minus2 = value;
  return *this;
}

auto GeometryAssistance::gas_bw(uint32_t value) noexcept -> GeometryAssistance & {
  m_gas_log2_bw_minus2 = Common::ceilLog2(value) - 2;
  return *this;
}

auto GeometryAssistance::blocks(std::vector<std::vector<std::vector<GaBlock>>> &value)
    -> GeometryAssistance & {
  m_gas_blocks = value;
  return *this;
}

auto operator<<(std::ostream &stream, const GeometryAssistance &x) -> std::ostream & {
  x.writeTo(stream);
  return stream;
}

auto GeometryAssistance::operator==(const GeometryAssistance &other) const noexcept -> bool {
  return gas_qs() == other.gas_qs() && gas_num_views() == other.gas_num_views() &&
         gas_bw() == other.gas_bw() &&
         gas_projection_plane_height_minus1() == other.gas_projection_plane_height_minus1() &&
         gas_projection_plane_width_minus1() == other.gas_projection_plane_width_minus1() &&
         blocks() == other.blocks();
}

auto GeometryAssistance::operator!=(const GeometryAssistance &other) const noexcept -> bool {
  return !operator==(other);
}

void GeometryAssistance::writeTo(std::ostream &stream) const {
  stream << "gas_qs=" << m_gas_qs << "\n";
  stream << "gas_num_views_minus1=" << m_gas_num_views_minus1 << "\n";
  stream << "gas_log2_bw_minus2=" << int32_t{m_gas_log2_bw_minus2} << "\n";
  uint16_t num_views = m_gas_num_views_minus1 + 1;
  for (uint16_t v_idx = 0; v_idx < num_views; v_idx++) {
    stream << "# VIEWIDX " << v_idx << "\n";
    stream << "gas_projection_plane_height_minus1[" << v_idx
           << "]=" << m_gas_projection_plane_height_minus1[v_idx] << "\n";
    stream << "gas_projection_plane_width_minus1[" << v_idx
           << "]=" << m_gas_projection_plane_width_minus1[v_idx] << "\n";
    auto v_h = m_gas_projection_plane_height_minus1[v_idx];
    auto v_h_in_blocks = (v_h + gas_bw() - 1) / gas_bw();
    auto v_w = m_gas_projection_plane_width_minus1[v_idx];
    auto v_w_in_blocks = (v_w + gas_bw() - 1) / gas_bw();
    for (decltype(v_h_in_blocks) blk_y = 0; blk_y < v_h_in_blocks; blk_y++) {
      for (decltype(v_w_in_blocks) blk_x = 0; blk_x < v_w_in_blocks; blk_x++) {
        stream << "block y=" << blk_y << " x=" << blk_x;
        m_gas_blocks[v_idx][blk_y][blk_x].writeTo(stream, blk_y, blk_x);
        stream << "\n";
      }
    }
  }
}

auto GeometryAssistance::readFrom(TMIV::Common::Json const &jin) -> GeometryAssistance {
  auto ga = GeometryAssistance{};

  ga.m_gas_qs = jin.require("gas_qs").as<int32_t>();
  ga.m_gas_num_views_minus1 = jin.require("gas_num_views_minus1").as<uint16_t>();
  ga.m_gas_log2_bw_minus2 = jin.require("gas_log2_bw_minus2").as<uint8_t>();

  uint16_t num_views_text = ga.m_gas_num_views_minus1 + 1;
  ga.m_gas_blocks.resize(num_views_text);
  ga.m_gas_projection_plane_height_minus1.resize(num_views_text);
  ga.m_gas_projection_plane_width_minus1.resize(num_views_text);

  for (uint16_t v_idx = 0; v_idx < num_views_text; v_idx++) {
    Common::Json const view_info = jin.require("view_idx_" + std::to_string(v_idx));
    ga.m_gas_projection_plane_height_minus1[v_idx] =
        view_info.require("gas_projection_plane_height_minus1").as<uint16_t>();
    ga.m_gas_projection_plane_width_minus1[v_idx] =
        view_info.require("gas_projection_plane_width_minus1").as<uint16_t>();
    auto v_h = ga.m_gas_projection_plane_height_minus1[v_idx] + 1;
    auto v_w = ga.m_gas_projection_plane_width_minus1[v_idx] + 1;
    auto v_h_in_blocks = (v_h + ga.gas_bw() - 1) / ga.gas_bw();
    auto v_w_in_blocks = (v_w + ga.gas_bw() - 1) / ga.gas_bw();
    ga.m_gas_blocks[v_idx].resize(v_h_in_blocks);
    auto yvec = view_info.require("blocks").as<Common::Json::Array>();
    if (yvec.size() != v_h_in_blocks) {
      throw std::runtime_error("GA: badly sized input y-vector");
    }
    for (decltype(v_h_in_blocks) blk_y = 0; blk_y < v_h_in_blocks; blk_y++) {
      ga.m_gas_blocks[v_idx][blk_y].resize(v_w_in_blocks);
      auto xvec = yvec.at(blk_y).as<Common::Json::Array>();
      if (xvec.size() != v_w_in_blocks) {
        throw std::runtime_error("GA: badly sized input x-vector");
      }
      for (decltype(v_w_in_blocks) blk_x = 0; blk_x < v_w_in_blocks; blk_x++) {
        ga.m_gas_blocks[v_idx][blk_y][blk_x].readFrom(xvec.at(blk_x), blk_y, blk_x);
      }
    }
  }

  return ga;
}

void GeometryAssistance::encodeTo(Common::OutputBitstream &bitstream) const {
  uint16_t num_views = m_gas_num_views_minus1 + 1;

  bitstream.putUExpGolomb(m_gas_qs);
  bitstream.putUExpGolomb(m_gas_num_views_minus1);
  bitstream.putUExpGolomb(m_gas_log2_bw_minus2);
  for (uint16_t v_idx = 0; v_idx < num_views; v_idx++) {
    bitstream.putUExpGolomb(m_gas_projection_plane_height_minus1[v_idx]);
    bitstream.putUExpGolomb(m_gas_projection_plane_width_minus1[v_idx]);
    auto v_h = m_gas_projection_plane_height_minus1[v_idx] + 1;
    auto v_h_in_blocks = (v_h + gas_bw() - 1) / gas_bw();
    auto v_w = m_gas_projection_plane_width_minus1[v_idx] + 1;
    auto v_w_in_blocks = (v_w + gas_bw() - 1) / gas_bw();
    for (decltype(v_h_in_blocks) blk_y = 0; blk_y < v_h_in_blocks; blk_y++) {
      for (decltype(v_h_in_blocks) blk_x = 0; blk_x < v_w_in_blocks; blk_x++) {
        m_gas_blocks[v_idx][blk_y][blk_x].encodeTo(bitstream, blk_y, blk_x);
      }
    }
  }
}

auto GeometryAssistance::decodeFrom(Common::InputBitstream &bitstream) -> GeometryAssistance {
  auto x = GeometryAssistance{};

  x.m_gas_qs = bitstream.getUExpGolomb<uint8_t>();
  x.m_gas_num_views_minus1 = bitstream.getUExpGolomb<uint16_t>();
  x.m_gas_log2_bw_minus2 = bitstream.getUExpGolomb<uint8_t>();

  uint16_t num_views = x.m_gas_num_views_minus1 + 1;
  x.m_gas_blocks.resize(num_views);
  x.m_gas_projection_plane_height_minus1.resize(num_views);
  x.m_gas_projection_plane_width_minus1.resize(num_views);

  for (uint16_t v_idx = 0; v_idx < num_views; v_idx++) {
    x.m_gas_projection_plane_height_minus1[v_idx] = bitstream.getUExpGolomb<uint16_t>();
    x.m_gas_projection_plane_width_minus1[v_idx] = bitstream.getUExpGolomb<uint16_t>();
    auto v_w = x.m_gas_projection_plane_width_minus1[v_idx] + 1;
    auto v_w_in_blocks = (v_w + x.gas_bw() - 1) / x.gas_bw();
    auto v_h = x.m_gas_projection_plane_height_minus1[v_idx] + 1;
    auto v_h_in_blocks = (v_h + x.gas_bw() - 1) / x.gas_bw();
    x.m_gas_blocks[v_idx].resize(v_h_in_blocks);
    for (decltype(v_h_in_blocks) blk_y = 0; blk_y < v_h_in_blocks; blk_y++) {
      x.m_gas_blocks[v_idx][blk_y].resize(v_w_in_blocks);
      for (decltype(v_h_in_blocks) blk_x = 0; blk_x < v_w_in_blocks; blk_x++) {
        x.m_gas_blocks[v_idx][blk_y][blk_x].decodeFrom(bitstream, blk_y, blk_x);
      }
    }
  }
  return x;
}
} // namespace TMIV::MivBitstream
