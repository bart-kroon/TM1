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

#include <TMIV/MivBitstream/PackedIndependentRegions.h>

#include <TMIV/MivBitstream/Formatters.h>

namespace TMIV::MivBitstream {
auto PackedIndependentRegions::pir_num_packed_frames_minus1() const -> uint8_t {
  VERIFY_V3CBITSTREAM(!m_pirPackedFrames.empty());
  return static_cast<uint8_t>(m_pirPackedFrames.size() - 1U);
}
auto PackedIndependentRegions::pir_packed_frame_id(uint8_t j) const -> uint8_t {
  VERIFY_V3CBITSTREAM(j <= pir_num_packed_frames_minus1());
  return m_pirPackedFrames[j].pir_packed_frame_id;
}
auto PackedIndependentRegions::pir_description_type_idc(uint8_t k) const -> uint8_t {
  VERIFY_V3CBITSTREAM(k <= pir_num_packed_frames_minus1());
  if (std::holds_alternative<TileRegions>(m_pirPackedFrames[k].regions)) {
    return 0U;
  }
  return 1U;
}
auto PackedIndependentRegions::pir_num_regions_minus1(uint8_t k) const -> uint8_t {
  VERIFY_V3CBITSTREAM(k <= pir_num_packed_frames_minus1());
  return static_cast<uint8_t>(
      std::visit([](const auto &regions) { return regions.size(); }, m_pirPackedFrames[k].regions) -
      1);
}
auto PackedIndependentRegions::pir_top_left_tile_idx(uint8_t k, uint8_t i) const -> size_t {
  VERIFY_V3CBITSTREAM(k <= pir_num_packed_frames_minus1() && pir_description_type_idc(k) == 0 &&
                      i <= pir_num_regions_minus1(k));
  const auto &tileRegions = std::get<TileRegions>(m_pirPackedFrames[k].regions);
  return tileRegions[i].pir_top_left_tile_idx;
}
auto PackedIndependentRegions::pir_bottom_right_tile_idx(uint8_t k, uint8_t i) const -> size_t {
  VERIFY_V3CBITSTREAM(k <= pir_num_packed_frames_minus1() && pir_description_type_idc(k) == 0 &&
                      i <= pir_num_regions_minus1(k));
  const auto &tileRegions = std::get<TileRegions>(m_pirPackedFrames[k].regions);
  return tileRegions[i].pir_bottom_right_tile_idx;
}
auto PackedIndependentRegions::pir_subpic_id(uint8_t k, uint8_t i) const -> size_t {
  VERIFY_V3CBITSTREAM(k <= pir_num_packed_frames_minus1() && pir_description_type_idc(k) == 1 &&
                      i <= pir_num_regions_minus1(k));
  const auto &subPicId = std::get<subPicIds>(m_pirPackedFrames[k].regions);
  return subPicId[i];
}

auto operator<<(std::ostream &stream, const PackedIndependentRegions &x) -> std::ostream & {
  TMIV_FMT::print(stream, "pir_num_packed_frames_minus1={}\n", x.pir_num_packed_frames_minus1());
  for (uint8_t j = 0; j <= x.pir_num_packed_frames_minus1(); ++j) {
    TMIV_FMT::print(stream, "pir_packed_frame_id[ {} ]={}\n", j, x.pir_packed_frame_id(j));
    const auto k = x.pir_packed_frame_id(j);
    TMIV_FMT::print(stream, "pir_description_type_idc[ {} ]={}\n", k,
                    x.pir_description_type_idc(k));
    TMIV_FMT::print(stream, "pir_num_regions_minus1[ {} ]={}\n", k, x.pir_num_regions_minus1(k));
    for (uint8_t i = 0; i <= x.pir_num_regions_minus1(k); ++i) {
      if (x.pir_description_type_idc(k) == 0) {
        TMIV_FMT::print(stream, "pir_top_left_tile_idx[ {} ][ {} ]={}\n", k, i,
                        x.pir_top_left_tile_idx(k, i));
        TMIV_FMT::print(stream, "pir_bottom_right_tile_idx[ {} ][ {} ]={}\n", k, i,
                        x.pir_bottom_right_tile_idx(k, i));
      } else {
        TMIV_FMT::print(stream, "pir_subpic_id[ {} ][ {} ]={}\n", k, i, x.pir_subpic_id(k, i));
      }
    }
  }
  return stream;
}

auto PackedIndependentRegions::operator==(const PackedIndependentRegions &other) const noexcept
    -> bool {
  return m_pirPackedFrames == other.m_pirPackedFrames;
}

auto PackedIndependentRegions::operator!=(const PackedIndependentRegions &other) const noexcept
    -> bool {
  return !operator==(other);
}

auto PackedIndependentRegions::decodeFrom(Common::InputBitstream &bitstream)
    -> PackedIndependentRegions {
  PackedIndependentRegions result{};
  result.pir_num_packed_frames_minus1(bitstream.readBits<uint8_t>(5));
  for (uint8_t j = 0; j <= result.pir_num_packed_frames_minus1(); ++j) {
    result.pir_packed_frame_id(j, bitstream.readBits<uint8_t>(5));
    const auto k = result.pir_packed_frame_id(j);
    result.pir_description_type_idc(k, bitstream.readBits<uint8_t>(2));
    result.pir_num_regions_minus1(k, bitstream.readBits<uint8_t>(8));
    for (uint8_t i = 0; i <= result.pir_num_regions_minus1(k); ++i) {
      if (result.pir_description_type_idc(k) == 0) {
        result.pir_top_left_tile_idx(k, i, bitstream.getUExpGolomb<size_t>());
        result.pir_bottom_right_tile_idx(k, i, bitstream.getUExpGolomb<size_t>());
      } else {
        result.pir_subpic_id(k, i, bitstream.getUExpGolomb<size_t>());
      }
    }
  }
  return result;
}

void PackedIndependentRegions::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.writeBits(pir_num_packed_frames_minus1(), 5);
  for (uint8_t j = 0; j <= pir_num_packed_frames_minus1(); ++j) {
    bitstream.writeBits(pir_packed_frame_id(j), 5);
    const auto k = pir_packed_frame_id(j);
    bitstream.writeBits(pir_description_type_idc(k), 2);
    bitstream.writeBits(pir_num_regions_minus1(k), 8);
    for (uint8_t i = 0; i <= pir_num_regions_minus1(k); ++i) {
      if (pir_description_type_idc(k) == 0) {
        bitstream.putUExpGolomb(pir_top_left_tile_idx(k, i));
        bitstream.putUExpGolomb(pir_bottom_right_tile_idx(k, i));
      } else {
        bitstream.putUExpGolomb(pir_subpic_id(k, i));
      }
    }
  }
}
} // namespace TMIV::MivBitstream
