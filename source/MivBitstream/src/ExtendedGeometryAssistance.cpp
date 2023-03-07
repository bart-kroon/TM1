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
#include <TMIV/MivBitstream/ExtendedGeometryAssistance.h>

namespace TMIV::MivBitstream {
enum ExtendedGeometryAssistanceType : uint8_t { block_based_geometry_features };

auto EgaBlock::operator==(const EgaBlock &other) const noexcept -> bool {
  return bbgf_split_flag() == other.bbgf_split_flag() &&
         bbgf_quad_split_flag() == other.bbgf_quad_split_flag() &&
         bbgf_split_orientation_flag() == other.bbgf_split_orientation_flag() &&
         bbgf_split_symmetry_flag() == other.bbgf_split_symmetry_flag() &&
         bbgf_split_first_block_bigger() == other.bbgf_split_first_block_bigger() &&
         subblocks() == other.subblocks();
}

auto EgaBlock::operator!=(const EgaBlock &other) const noexcept -> bool {
  return !operator==(other);
}

auto EgaBlock::bbgf_split_flag() const noexcept -> bool { return m_bbgf_split_flag; }

auto EgaBlock::bbgf_quad_split_flag() const noexcept -> bool { return m_bbgf_quad_split_flag; }

auto EgaBlock::bbgf_split_orientation_flag() const noexcept -> bool {
  return m_bbgf_split_orientation_flag;
}

auto EgaBlock::bbgf_split_symmetry_flag() const noexcept -> bool {
  return m_bbgf_split_symmetry_flag;
}

auto EgaBlock::bbgf_split_first_block_bigger() const noexcept -> bool {
  return m_bbgf_split_first_block_bigger;
}

auto EgaBlock::subblocks() const noexcept -> const std::vector<EgaBlock> & { return m_sub_blocks; }

auto EgaBlock::bbgf_split_flag(bool value) noexcept -> EgaBlock & {
  m_bbgf_split_flag = value;
  return *this;
}

auto EgaBlock::bbgf_quad_split_flag(bool value) noexcept -> EgaBlock & {
  m_bbgf_quad_split_flag = value;
  return *this;
}

auto EgaBlock::bbgf_split_orientation_flag(bool value) noexcept -> EgaBlock & {
  m_bbgf_split_orientation_flag = value;
  return *this;
}

auto EgaBlock::bbgf_split_symmetry_flag(bool value) noexcept -> EgaBlock & {
  m_bbgf_split_symmetry_flag = value;
  return *this;
}

auto EgaBlock::bbgf_split_first_block_bigger(bool value) noexcept -> EgaBlock & {
  m_bbgf_split_first_block_bigger = value;
  return *this;
}
auto EgaBlock::subblocks(std::vector<EgaBlock> &value) -> EgaBlock & {
  m_sub_blocks = value;
  return *this;
}

auto EgaBlock::writeTo(std::ostream &stream, const uint32_t bbgf_max_number_of_splits,
                       uint32_t blk_y, uint32_t blk_x, uint32_t split_lvl) const -> std::ostream & {
  uint32_t n_sb_x = 0;
  uint32_t n_sb_y = 0;
  if (split_lvl < bbgf_max_number_of_splits) {
    stream << " bbgf_split_flag=" << m_bbgf_split_flag;
  }
  if (split_lvl < bbgf_max_number_of_splits && m_bbgf_split_flag) {
    stream << " bbgf_quad_split_flag=" << m_bbgf_quad_split_flag;
    if (m_bbgf_quad_split_flag) {
      n_sb_x = n_sb_y = 2;
    } else {
      stream << " bbgf_split_orientation_flag=" << m_bbgf_split_orientation_flag;
      stream << " bbgf_split_symmetry_flag=" << m_bbgf_split_symmetry_flag;
      if (!m_bbgf_split_symmetry_flag) {
        stream << " bbgf_split_first_block_bigger=" << m_bbgf_split_first_block_bigger;
      }
      if (m_bbgf_split_orientation_flag) {
        n_sb_x = 2;
        n_sb_y = 1;
      } else {
        n_sb_x = 1;
        n_sb_y = 2;
      }
    }
    uint32_t sb_idx = 0;
    for (uint32_t sb_y = 0; sb_y < n_sb_y; sb_y++) {
      for (uint32_t sb_x = 0; sb_x < n_sb_x; sb_x++, sb_idx++) {
        stream << " [";
        m_sub_blocks[sb_idx].writeTo(stream, bbgf_max_number_of_splits, blk_y + sb_y, blk_x + sb_x,
                                     split_lvl + 1);
        stream << " ]";
      }
    }
  } else {
    stream << " bbgf_skip_flag=" << m_bbgf_skip_flag;
    if (!m_bbgf_skip_flag) {
      if (blk_y == 0 || blk_x == 0) {
        ; // no neighbor signalling
      } else {
        stream << " bbgf_ltmin_flag=" << m_bbgf_ltmin_flag;
        stream << " bbgf_ltmax_flag=" << m_bbgf_ltmax_flag;
      }
      stream << " bbgf_zmin_delta=" << m_bbgf_zmin_delta;
      stream << " bbgf_zmax_delta=" << m_bbgf_zmax_delta;
    }
  }
  return stream;
}

void EgaBlock::readFrom(TMIV::Common::Json const &jin, const uint32_t bbgf_max_number_of_splits,
                        uint32_t blk_y, uint32_t blk_x, uint32_t split_lvl) {
  uint32_t n_sb_x = 1;
  uint32_t n_sb_y = 1;
  if (split_lvl < bbgf_max_number_of_splits) {
    m_bbgf_split_flag = jin.require("bbgf_split_flag").as<int32_t>() != 0;
  }
  if (split_lvl < bbgf_max_number_of_splits && m_bbgf_split_flag) {
    m_bbgf_quad_split_flag = jin.require("bbgf_quad_split_flag").as<int32_t>() != 0;
    if (m_bbgf_quad_split_flag) {
      n_sb_x = n_sb_y = 2;
    } else {
      m_bbgf_split_orientation_flag = jin.require("bbgf_split_orientation_flag").as<int32_t>() != 0;
      m_bbgf_split_symmetry_flag = jin.require("bbgf_split_symmetry_flag").as<int32_t>() != 0;
      if (!m_bbgf_split_symmetry_flag) {
        m_bbgf_split_first_block_bigger =
            jin.require("bbgf_split_first_block_bigger").as<int32_t>() != 0;
      }
      if (m_bbgf_split_orientation_flag) {
        n_sb_x = 2;
        n_sb_y = 1;
      } else {
        n_sb_x = 1;
        n_sb_y = 2;
      }
    }
    uint32_t sb_idx = 0;
    auto subblks = jin.require("subblks").as<Common::Json::Array>();
    m_sub_blocks.resize(n_sb_y * n_sb_x);
    for (uint32_t sb_y = 0; sb_y < n_sb_y; sb_y++) {
      for (uint32_t sb_x = 0; sb_x < n_sb_x; sb_x++, sb_idx++) {
        m_sub_blocks[sb_idx].readFrom(subblks.at(sb_idx), bbgf_max_number_of_splits, blk_y + sb_y,
                                      blk_x + sb_x, split_lvl + 1);
      }
    }
  } else {
    m_bbgf_skip_flag = jin.require("bbgf_skip_flag").as<int32_t>() != 0;
    if (!m_bbgf_skip_flag) {
      if (blk_y == 0 || blk_x == 0) {
        ; // no neighbor signalling
      } else {
        m_bbgf_ltmin_flag = jin.require("bbgf_ltmin_flag").as<int32_t>() != 0;
        m_bbgf_ltmax_flag = jin.require("bbgf_ltmax_flag").as<int32_t>() != 0;
      }
      m_bbgf_zmin_delta = jin.require("bbgf_zmin_delta").as<int32_t>();
      m_bbgf_zmax_delta = jin.require("bbgf_zmax_delta").as<int32_t>();
    }
  }
}

void EgaBlock::encodeTo(Common::OutputBitstream &bitstream,
                        const uint32_t bbgf_max_number_of_splits, uint32_t blk_y, uint32_t blk_x,
                        uint32_t split_lvl) const {
  uint32_t n_sb_x = 1;
  uint32_t n_sb_y = 1;
  if (split_lvl < bbgf_max_number_of_splits) {
    bitstream.putFlag(m_bbgf_split_flag);
  }
  if (split_lvl < bbgf_max_number_of_splits && m_bbgf_split_flag) {
    bitstream.putFlag(m_bbgf_quad_split_flag);
    if (m_bbgf_quad_split_flag) {
      n_sb_x = n_sb_y = 2;
    } else {
      bitstream.putFlag(m_bbgf_split_orientation_flag);
      bitstream.putFlag(m_bbgf_split_symmetry_flag);
      if (!m_bbgf_split_symmetry_flag) {
        bitstream.putFlag(m_bbgf_split_first_block_bigger);
      }
      if (m_bbgf_split_orientation_flag) {
        n_sb_x = 2;
      } else {
        n_sb_y = 2;
      }
    }
    uint32_t sb_idx = 0;
    for (uint32_t sb_y = 0; sb_y < n_sb_y; sb_y++) {
      for (uint32_t sb_x = 0; sb_x < n_sb_x; sb_x++, sb_idx++) {
        m_sub_blocks[sb_idx].encodeTo(bitstream, bbgf_max_number_of_splits, blk_y + sb_y,
                                      blk_x + sb_x, split_lvl + 1);
      }
    }
  } else {
    bitstream.putFlag(m_bbgf_skip_flag);
    if (!m_bbgf_skip_flag) {
      if (blk_y == 0 || blk_x == 0) {
        ; // no neighbor signalling
      } else {
        bitstream.putFlag(m_bbgf_ltmin_flag);
        bitstream.putFlag(m_bbgf_ltmax_flag);
      }
      bitstream.putSExpGolomb(m_bbgf_zmin_delta);
      bitstream.putSExpGolomb(m_bbgf_zmax_delta);
    }
  }
}

void EgaBlock::decodeFrom(Common::InputBitstream &bitstream,
                          const uint32_t bbgf_max_number_of_splits, uint32_t blk_y, uint32_t blk_x,
                          uint32_t split_lvl) {
  uint32_t n_sb_x = 1;
  uint32_t n_sb_y = 1;
  if (split_lvl < bbgf_max_number_of_splits) {
    m_bbgf_split_flag = bitstream.getFlag();
  }
  if (split_lvl < bbgf_max_number_of_splits && m_bbgf_split_flag) {
    m_bbgf_quad_split_flag = bitstream.getFlag();
    if (m_bbgf_quad_split_flag) {
      n_sb_x = n_sb_y = 2;
    } else {
      m_bbgf_split_orientation_flag = bitstream.getFlag();
      m_bbgf_split_symmetry_flag = bitstream.getFlag();
      if (!m_bbgf_split_symmetry_flag) {
        m_bbgf_split_first_block_bigger = bitstream.getFlag();
      }
      if (m_bbgf_split_orientation_flag) {
        n_sb_x = 2;
      } else {
        n_sb_y = 2;
      }
    }
    uint32_t sb_idx = 0;
    m_sub_blocks.resize(n_sb_y * n_sb_x);
    for (uint32_t sb_y = 0; sb_y < n_sb_y; sb_y++) {
      for (uint32_t sb_x = 0; sb_x < n_sb_x; sb_x++, sb_idx++) {
        m_sub_blocks[sb_idx].decodeFrom(bitstream, bbgf_max_number_of_splits, blk_y + sb_y,
                                        blk_x + sb_x, split_lvl + 1);
      }
    }
  } else {
    m_bbgf_skip_flag = bitstream.getFlag();
    if (!m_bbgf_skip_flag) {
      if (blk_y == 0 || blk_x == 0) {
        ; // no neighbor signalling
      } else {
        m_bbgf_ltmin_flag = bitstream.getFlag();
        m_bbgf_ltmax_flag = bitstream.getFlag();
      }
      m_bbgf_zmin_delta = bitstream.getSExpGolomb<int32_t>();
      m_bbgf_zmax_delta = bitstream.getSExpGolomb<int32_t>();
    }
  }
}

auto EgaBlockBasedGeometryFeatures::operator==(
    const EgaBlockBasedGeometryFeatures &other) const noexcept -> bool {
  return bbgf_qs() == other.bbgf_qs() && bbgf_bw() == other.bbgf_bw() &&
         bbgf_max_number_of_splits() == other.bbgf_max_number_of_splits() &&
         bbgf_projection_plane_height_minus1() == other.bbgf_projection_plane_height_minus1() &&
         bbgf_projection_plane_width_minus1() == other.bbgf_projection_plane_width_minus1() &&
         initial_block_grid() == other.initial_block_grid();
}

auto EgaBlockBasedGeometryFeatures::operator!=(
    const EgaBlockBasedGeometryFeatures &other) const noexcept -> bool {
  return !operator==(other);
}

auto EgaBlockBasedGeometryFeatures::bbgf_qs() const noexcept -> uint32_t { return m_bbgf_qs; }

auto EgaBlockBasedGeometryFeatures::bbgf_bw() const noexcept -> uint32_t {
  uint32_t result = 1 << (m_bbgf_log2_bw_minus2 + 2);
  return result;
}

auto EgaBlockBasedGeometryFeatures::bbgf_max_number_of_splits() const noexcept -> uint8_t {
  return m_bbgf_max_number_of_splits;
}

auto EgaBlockBasedGeometryFeatures::bbgf_projection_plane_height_minus1() const noexcept
    -> uint16_t {
  return m_bbgf_projection_plane_height_minus1;
}

auto EgaBlockBasedGeometryFeatures::bbgf_projection_plane_width_minus1() const noexcept
    -> uint16_t {
  return m_bbgf_projection_plane_width_minus1;
}

auto EgaBlockBasedGeometryFeatures::initial_block_grid() const noexcept
    -> const std::vector<std::vector<EgaBlock>> & {
  return m_bbgf_initial_block_grid;
}

auto EgaBlockBasedGeometryFeatures::bbgf_qs(uint32_t value) noexcept
    -> EgaBlockBasedGeometryFeatures & {
  m_bbgf_qs = value;
  return *this;
}

auto EgaBlockBasedGeometryFeatures::bbgf_bw(uint32_t value) noexcept
    -> EgaBlockBasedGeometryFeatures & {
  m_bbgf_log2_bw_minus2 = Common::ceilLog2(value) - 2;
  return *this;
}

auto EgaBlockBasedGeometryFeatures::bbgf_log2_bw_minus2(uint8_t value) noexcept
    -> EgaBlockBasedGeometryFeatures & {
  m_bbgf_log2_bw_minus2 = value;
  return *this;
}

auto EgaBlockBasedGeometryFeatures::bbgf_max_number_of_splits(uint8_t value) noexcept
    -> EgaBlockBasedGeometryFeatures & {
  m_bbgf_max_number_of_splits = value;
  return *this;
}

auto EgaBlockBasedGeometryFeatures::writeTo(std::ostream &stream, uint32_t v_idx) const
    -> std::ostream & {
  stream << "bbgf_qs[" << v_idx << "]=" << m_bbgf_qs << "\n";
  stream << "bbgf_log2_bw_minus2[" << v_idx << "]=" << static_cast<uint32_t>(m_bbgf_log2_bw_minus2)
         << "\n";
  stream << "bbgf_max_number_of_splits[" << v_idx
         << "]=" << static_cast<uint32_t>(m_bbgf_max_number_of_splits) << "\n";
  stream << "bbgf_projection_plane_height_minus1[" << v_idx
         << "]=" << m_bbgf_projection_plane_height_minus1 << "\n";
  stream << "bbgf_projection_plane_width_minus1[" << v_idx
         << "]=" << m_bbgf_projection_plane_width_minus1 << "\n";

  auto v_w = m_bbgf_projection_plane_width_minus1 + 1;
  auto v_w_in_blocks = (v_w + bbgf_bw() - 1) / bbgf_bw();
  auto v_h = m_bbgf_projection_plane_height_minus1 + 1;
  auto v_h_in_blocks = (v_h + bbgf_bw() - 1) / bbgf_bw();

  for (decltype(v_h_in_blocks) blk_y = 0; blk_y < v_h_in_blocks; blk_y++) {
    for (decltype(v_w_in_blocks) blk_x = 0; blk_x < v_w_in_blocks; blk_x++) {
      stream << "block y=" << blk_y << " x=" << blk_x;
      m_bbgf_initial_block_grid[blk_y][blk_x].writeTo(stream, m_bbgf_max_number_of_splits, blk_y,
                                                      blk_x, 0);
      stream << "\n";
    }
  }
  return stream;
}

void EgaBlockBasedGeometryFeatures::readFrom(Common::Json const &jin) {
  m_bbgf_qs = jin.require("bbgf_qs").as<uint32_t>();
  m_bbgf_log2_bw_minus2 = jin.require("bbgf_log2_bw_minus2").as<uint8_t>();
  m_bbgf_max_number_of_splits = jin.require("bbgf_max_number_of_splits").as<uint8_t>();
  m_bbgf_projection_plane_height_minus1 =
      jin.require("bbgf_projection_plane_height_minus1").as<uint16_t>();
  m_bbgf_projection_plane_width_minus1 =
      jin.require("bbgf_projection_plane_width_minus1").as<uint16_t>();

  auto v_w = m_bbgf_projection_plane_width_minus1 + 1;
  auto v_w_in_blocks = (v_w + bbgf_bw() - 1) / bbgf_bw();
  auto v_h = m_bbgf_projection_plane_height_minus1 + 1;
  auto v_h_in_blocks = (v_h + bbgf_bw() - 1) / bbgf_bw();

  m_bbgf_initial_block_grid.resize(v_h_in_blocks);
  auto yvec = jin.require("blocks").as<Common::Json::Array>();
  if (yvec.size() != v_h_in_blocks) {
    throw std::runtime_error("EGA: badly sized input y-vector");
  }
  for (decltype(v_h_in_blocks) blk_y = 0; blk_y < v_h_in_blocks; blk_y++) {
    m_bbgf_initial_block_grid[blk_y].resize(v_w_in_blocks);
    auto xvec = yvec.at(blk_y).as<Common::Json::Array>();
    if (xvec.size() != v_w_in_blocks) {
      throw std::runtime_error("EGA: badly sized input x-vector");
    }
    for (decltype(v_w_in_blocks) blk_x = 0; blk_x < v_w_in_blocks; blk_x++) {
      m_bbgf_initial_block_grid[blk_y][blk_x].readFrom(xvec.at(blk_x), m_bbgf_max_number_of_splits,
                                                       blk_y, blk_x, 0);
    }
  }
}

void EgaBlockBasedGeometryFeatures::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUExpGolomb(m_bbgf_qs);
  bitstream.putUExpGolomb(m_bbgf_log2_bw_minus2);
  bitstream.putUExpGolomb(m_bbgf_max_number_of_splits);
  bitstream.putUExpGolomb(m_bbgf_projection_plane_height_minus1);
  bitstream.putUExpGolomb(m_bbgf_projection_plane_width_minus1);

  auto v_w = m_bbgf_projection_plane_width_minus1 + 1;
  auto v_w_in_blocks = (v_w + bbgf_bw() - 1) / bbgf_bw();
  auto v_h = m_bbgf_projection_plane_height_minus1 + 1;
  auto v_h_in_blocks = (v_h + bbgf_bw() - 1) / bbgf_bw();

  for (decltype(v_h_in_blocks) blk_y = 0; blk_y < v_h_in_blocks; blk_y++) {
    for (decltype(v_w_in_blocks) blk_x = 0; blk_x < v_w_in_blocks; blk_x++) {
      m_bbgf_initial_block_grid[blk_y][blk_x].encodeTo(bitstream, m_bbgf_max_number_of_splits,
                                                       blk_y, blk_x, 0);
    }
  }
}

void EgaBlockBasedGeometryFeatures::decodeFrom(Common::InputBitstream &bitstream) {
  m_bbgf_qs = bitstream.getUExpGolomb<uint32_t>();
  m_bbgf_log2_bw_minus2 = bitstream.getUExpGolomb<uint8_t>();
  m_bbgf_max_number_of_splits = bitstream.getUExpGolomb<uint8_t>();
  m_bbgf_projection_plane_height_minus1 = bitstream.getUExpGolomb<uint16_t>();
  m_bbgf_projection_plane_width_minus1 = bitstream.getUExpGolomb<uint16_t>();

  auto v_w = m_bbgf_projection_plane_width_minus1 + 1;
  auto v_w_in_blocks = (v_w + bbgf_bw() - 1) / bbgf_bw();
  auto v_h = m_bbgf_projection_plane_height_minus1 + 1;
  auto v_h_in_blocks = (v_h + bbgf_bw() - 1) / bbgf_bw();

  m_bbgf_initial_block_grid.resize(v_h_in_blocks);
  for (decltype(v_h_in_blocks) blk_y = 0; blk_y < v_h_in_blocks; blk_y++) {
    m_bbgf_initial_block_grid[blk_y].resize(v_w_in_blocks);
    for (decltype(v_w_in_blocks) blk_x = 0; blk_x < v_w_in_blocks; blk_x++) {
      m_bbgf_initial_block_grid[blk_y][blk_x].decodeFrom(bitstream, m_bbgf_max_number_of_splits,
                                                         blk_y, blk_x, 0);
    }
  }
}

auto operator<<(std::ostream &stream, const ExtendedGeometryAssistance &x) -> std::ostream & {
  x.writeTo(stream);
  return stream;
}

auto ExtendedGeometryAssistance::operator==(const ExtendedGeometryAssistance &other) const noexcept
    -> bool {
  return ega_num_views() == other.ega_num_views() &&
         ega_num_available_assistance_types_minus1() ==
             other.ega_num_available_assistance_types_minus1() &&
         ega_assistance_present_flag() == other.ega_assistance_present_flag() &&
         ega_assistance_type_present_flag() == other.ega_assistance_type_present_flag() &&
         block_based_geometry_features() == other.block_based_geometry_features();
}

auto ExtendedGeometryAssistance::operator!=(const ExtendedGeometryAssistance &other) const noexcept
    -> bool {
  return !operator==(other);
}

auto ExtendedGeometryAssistance::ega_num_views() const noexcept -> uint16_t {
  return m_ega_num_views_minus1 + 1;
}

auto ExtendedGeometryAssistance::ega_num_available_assistance_types_minus1() const noexcept
    -> uint8_t {
  return m_ega_num_available_assistance_types_minus1;
}

auto ExtendedGeometryAssistance::ega_assistance_present_flag() const noexcept
    -> const std::vector<bool> & {
  return m_ega_assistance_present_flag;
}
auto ExtendedGeometryAssistance::ega_assistance_type_present_flag() const noexcept
    -> const std::vector<std::vector<bool>> & {
  return m_ega_assistance_type_present_flag;
}

auto ExtendedGeometryAssistance::block_based_geometry_features() const noexcept
    -> const std::vector<EgaBlockBasedGeometryFeatures> & {
  return m_ega_block_based_geometry_features;
}

auto ExtendedGeometryAssistance::ega_num_views_minus1(uint16_t value) noexcept
    -> ExtendedGeometryAssistance & {
  m_ega_num_views_minus1 = value;
  return *this;
}

auto ExtendedGeometryAssistance::ega_num_available_assistance_types_minus1(uint8_t value) noexcept
    -> ExtendedGeometryAssistance & {
  m_ega_num_available_assistance_types_minus1 = value;
  return *this;
}

void ExtendedGeometryAssistance::writeTo(std::ostream &stream) const {
  stream << "ega_num_views_minus1=" << m_ega_num_views_minus1 << "\n";
  stream << "ega_num_available_assistance_types_minus1="
         << static_cast<uint32_t>(m_ega_num_available_assistance_types_minus1) << "\n";
  uint16_t num_views = m_ega_num_views_minus1 + 1;
  uint8_t num_types = m_ega_num_available_assistance_types_minus1 + 1;

  for (uint16_t v_idx = 0; v_idx < num_views; v_idx++) {
    stream << "# VIEWIDX " << v_idx << "\n";
    stream << "ega_assistance_present_flag[" << v_idx
           << "]=" << m_ega_assistance_present_flag[v_idx] << "\n";
    if (m_ega_assistance_present_flag[v_idx]) {
      for (uint8_t t_idx = 0; t_idx < num_types; t_idx++) {
        stream << "# EGATYPEIDX " << static_cast<uint32_t>(t_idx) << "\n";
        stream << "ega_assistance_type_present_flag[" << v_idx << "]["
               << static_cast<uint32_t>(t_idx)
               << "]=" << m_ega_assistance_type_present_flag[v_idx][t_idx] << "\n";
        switch (t_idx) {
        case ExtendedGeometryAssistanceType::block_based_geometry_features:
          if (m_ega_assistance_type_present_flag[v_idx][t_idx]) {
            m_ega_block_based_geometry_features[v_idx].writeTo(stream, v_idx);
          }
          break;
        }
      }
    }
  }
}

auto ExtendedGeometryAssistance::readFrom(Common::Json const &jin) -> ExtendedGeometryAssistance {
  auto ega = ExtendedGeometryAssistance{};

  ega.m_ega_num_views_minus1 = jin.require("ega_num_views_minus1").as<uint16_t>();
  ega.m_ega_num_available_assistance_types_minus1 =
      jin.require("ega_num_available_assistance_types_minus1").as<uint8_t>();

  uint16_t num_views = ega.m_ega_num_views_minus1 + 1;
  uint8_t num_types = ega.m_ega_num_available_assistance_types_minus1 + 1;

  ega.m_ega_assistance_present_flag.resize(num_views);
  ega.m_ega_assistance_type_present_flag.resize(num_views);
  ega.m_ega_block_based_geometry_features.resize(num_views);

  for (uint16_t v_idx = 0; v_idx < num_views; v_idx++) {
    Common::Json const view_info = jin.require("view_idx_" + std::to_string(v_idx));
    ega.m_ega_assistance_present_flag[v_idx] =
        view_info.require("ega_assistance_present_flag").as<int32_t>() != 0;
    if (ega.m_ega_assistance_present_flag[v_idx]) {
      ega.m_ega_assistance_type_present_flag[v_idx].resize(num_types);
      for (uint8_t t_idx = 0; t_idx < num_types; t_idx++) {
        Common::Json const ega_type_info =
            view_info.require("ega_type_idx_" + std::to_string(t_idx));
        ega.m_ega_assistance_type_present_flag[v_idx][t_idx] =
            ega_type_info.require("ega_assistance_type_present_flag").as<int32_t>() != 0;
        switch (t_idx) {
        case ExtendedGeometryAssistanceType::block_based_geometry_features:
          if (ega.m_ega_assistance_type_present_flag[v_idx][t_idx]) {
            ega.m_ega_block_based_geometry_features[v_idx].readFrom(ega_type_info);
          }
          break;
        }
      }
    }
  }
  return ega;
}

void ExtendedGeometryAssistance::encodeTo(Common::OutputBitstream &bitstream) const {
  uint16_t num_views = m_ega_num_views_minus1 + 1;
  uint8_t num_types = m_ega_num_available_assistance_types_minus1 + 1;

  bitstream.putUExpGolomb(m_ega_num_views_minus1);
  bitstream.putUExpGolomb(m_ega_num_available_assistance_types_minus1);

  for (uint16_t v_idx = 0; v_idx < num_views; v_idx++) {
    bitstream.putFlag(m_ega_assistance_present_flag[v_idx]);
    if (m_ega_assistance_present_flag[v_idx]) {
      for (uint8_t t_idx = 0; t_idx < num_types; t_idx++) {
        bitstream.putFlag(m_ega_assistance_type_present_flag[v_idx][t_idx]);
        switch (t_idx) {
        case ExtendedGeometryAssistanceType::block_based_geometry_features:
          if (m_ega_assistance_type_present_flag[v_idx][t_idx]) {
            m_ega_block_based_geometry_features[v_idx].encodeTo(bitstream);
          }
          break;
        }
      }
    }
  }
}

auto ExtendedGeometryAssistance::decodeFrom(Common::InputBitstream &bitstream)
    -> ExtendedGeometryAssistance {
  auto x = ExtendedGeometryAssistance{};

  x.m_ega_num_views_minus1 = bitstream.getUExpGolomb<uint16_t>();
  x.m_ega_num_available_assistance_types_minus1 = bitstream.getUExpGolomb<uint8_t>();

  uint16_t num_views = x.m_ega_num_views_minus1 + 1;
  uint8_t num_types = x.m_ega_num_available_assistance_types_minus1 + 1;

  x.m_ega_assistance_present_flag.resize(num_views);
  x.m_ega_assistance_type_present_flag.resize(num_views);
  x.m_ega_block_based_geometry_features.resize(num_views);

  for (uint16_t v_idx = 0; v_idx < num_views; v_idx++) {
    x.m_ega_assistance_present_flag[v_idx] = bitstream.getFlag();
    if (x.m_ega_assistance_present_flag[v_idx]) {
      x.m_ega_assistance_type_present_flag[v_idx].resize(num_types);
      for (uint8_t t_idx = 0; t_idx < num_types; t_idx++) {
        x.m_ega_assistance_type_present_flag[v_idx][t_idx] = bitstream.getFlag();
        switch (t_idx) {
        case ExtendedGeometryAssistanceType::block_based_geometry_features:
          if (x.m_ega_assistance_type_present_flag[v_idx][t_idx]) {
            x.m_ega_block_based_geometry_features[v_idx].decodeFrom(bitstream);
          }
          break;
        }
      }
    }
  }
  return x;
}
} // namespace TMIV::MivBitstream