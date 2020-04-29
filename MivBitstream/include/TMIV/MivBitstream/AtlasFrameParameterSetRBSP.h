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

#ifndef _TMIV_MIVBITSTREAM_ATLASFRAMEPARAMETERSETRBSP_H_
#define _TMIV_MIVBITSTREAM_ATLASFRAMEPARAMETERSETRBSP_H_

#include <TMIV/Common/Bitstream.h>
#include <TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h>

#include <cstdint>
#include <cstdlib>
#include <iosfwd>

namespace TMIV::MivBitstream {
// 23090-5: atlas_frame_tile_information()
class AtlasFrameTileInformation {
public:
  friend auto operator<<(std::ostream &stream, const AtlasFrameTileInformation &x)
      -> std::ostream &;

  [[nodiscard]] constexpr auto afti_single_tile_in_atlas_frame_flag() const noexcept {
    return true;
  }

  auto operator==(const AtlasFrameTileInformation &) const noexcept -> bool { return true; }
  auto operator!=(const AtlasFrameTileInformation &) const noexcept -> bool { return false; }

  static auto decodeFrom(Common::InputBitstream &bitstream) -> AtlasFrameTileInformation;

  static void encodeTo(Common::OutputBitstream &stream);
};

// 23090-5: atlas_frame_parameter_set_rbsp()
class AtlasFrameParameterSetRBSP {
public:
  friend auto operator<<(std::ostream &stream, const AtlasFrameParameterSetRBSP &x)
      -> std::ostream &;

  [[nodiscard]] constexpr auto afps_atlas_frame_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto afps_atlas_sequence_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto atlas_frame_tile_information() const noexcept;
  [[nodiscard]] constexpr auto afps_output_flag_present_flag() const noexcept;
  [[nodiscard]] constexpr auto afps_num_ref_idx_default_active_minus1() const noexcept;
  [[nodiscard]] constexpr auto afps_additional_lt_afoc_lsb_len() const noexcept;
  [[nodiscard]] constexpr auto afps_3d_pos_x_bit_count_minus1() const noexcept;
  [[nodiscard]] constexpr auto afps_3d_pos_y_bit_count_minus1() const noexcept;
  [[nodiscard]] constexpr auto afps_lod_mode_enabled_flag() const noexcept;
  [[nodiscard]] constexpr auto afps_override_eom_for_depth_flag() const noexcept;
  [[nodiscard]] constexpr auto afps_raw_3d_pos_bit_count_explicit_mode_flag() const noexcept;
  [[nodiscard]] constexpr auto afps_extension_present_flag() const noexcept;

  constexpr auto afps_atlas_frame_parameter_set_id(const std::uint8_t value) noexcept -> auto &;
  constexpr auto afps_atlas_sequence_parameter_set_id(const std::uint8_t value) noexcept -> auto &;
  constexpr auto atlas_frame_tile_information(const AtlasFrameTileInformation &value) noexcept
      -> auto &;
  constexpr auto afps_output_flag_present_flag(const bool value) noexcept -> auto &;
  constexpr auto afps_num_ref_idx_default_active_minus1(const std::uint8_t value) noexcept
      -> auto &;
  constexpr auto afps_additional_lt_afoc_lsb_len(const std::uint8_t value) noexcept -> auto &;
  constexpr auto afps_3d_pos_x_bit_count_minus1(const std::uint8_t value) noexcept -> auto &;
  constexpr auto afps_3d_pos_y_bit_count_minus1(const std::uint8_t value) noexcept -> auto &;
  constexpr auto afps_lod_mode_enabled_flag(const bool value) noexcept -> auto &;
  constexpr auto afps_override_eom_for_depth_flag(const bool value) noexcept -> auto &;
  constexpr auto afps_raw_3d_pos_bit_count_explicit_mode_flag(const bool value) noexcept -> auto &;
  constexpr auto afps_extension_present_flag(const bool value) noexcept -> auto &;

  auto operator==(const AtlasFrameParameterSetRBSP &other) const noexcept -> bool;
  auto operator!=(const AtlasFrameParameterSetRBSP &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream,
                         const std::vector<AtlasSequenceParameterSetRBSP> &aspsV)
      -> AtlasFrameParameterSetRBSP;

  void encodeTo(std::ostream &stream,
                const std::vector<AtlasSequenceParameterSetRBSP> &aspsV) const;

private:
  std::uint8_t m_afps_atlas_frame_parameter_set_id{};
  std::uint8_t m_afps_atlas_sequence_parameter_set_id{};
  AtlasFrameTileInformation m_atlas_frame_tile_information;
  bool m_afps_output_flag_present_flag{};
  std::uint8_t m_afps_num_ref_idx_default_active_minus1{};
  std::uint8_t m_afps_additional_lt_afoc_lsb_len{};
  std::uint8_t m_afps_3d_pos_x_bit_count_minus1{};
  std::uint8_t m_afps_3d_pos_y_bit_count_minus1{};
  bool m_afps_lod_enabled_flag{};
  bool m_afps_override_eom_for_depth_flag{};
  bool m_afps_raw_3d_pos_bit_count_explicit_mode_flag{};
  bool m_afps_extension_present_flag{};
};
} // namespace TMIV::MivBitstream

#include "AtlasFrameParameterSetRBSP.hpp"

#endif
