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

#ifndef _TMIV_MIVBITSTREAM_MIVPARAMETERSET_H_
#define _TMIV_MIVBITSTREAM_MIVPARAMETERSET_H_

#include <TMIV/Metadata/IvSequenceParams.h>
#include <TMIV/VpccBitstream/VpccParameterSet.h>

namespace TMIV::MivBitstream {
// NOTE(BK): The num values are just placeholders. I believe that we should discuss profiles during
// the MPEG 129 meeting. The V-PCC profile_tier_level structure is also present and the
// msp_profile_idc interacts with that. (In some way to be defined.)
enum class MspProfileIdc : std::uint8_t { Basic, Extended };

auto operator<<(std::ostream &stream, MspProfileIdc x) -> std::ostream &;

// 23090-12 proposal: miv_sequence_params (based on WD3 iv_sequence_params)
class MivSequenceParams {
public:
  constexpr auto msp_profile_idc() const noexcept;
  constexpr auto msp_depth_params_num_bits() const noexcept;
  constexpr auto &view_params_list() const noexcept;
  constexpr auto msp_depth_low_quality_flag() const noexcept;
  constexpr auto msp_num_groups() const noexcept;
  constexpr auto msp_max_entities() const noexcept;
  constexpr auto msp_viewing_space_present_flag() const noexcept;
  auto viewing_space() const noexcept -> const Metadata::ViewingSpace &;
  constexpr auto msp_extension_present_flag() const noexcept;

  constexpr auto &msp_profile_idc(MspProfileIdc value) noexcept;
  constexpr auto &msp_depth_params_num_bits(std::uint8_t value) noexcept;
  auto view_params_list(Metadata::ViewParamsList value) -> MivSequenceParams &;
  constexpr auto &msp_depth_low_quality_flag(bool value) noexcept;
  constexpr auto &msp_num_groups(std::size_t value) noexcept;
  constexpr auto &msp_max_entities(std::size_t value) noexcept;
  auto msp_viewing_space_present_flag(bool value) noexcept -> MivSequenceParams &;
  auto viewing_space(Metadata::ViewingSpace value) noexcept -> MivSequenceParams &;
  constexpr auto &msp_extension_present_flag(bool value) noexcept;

  auto viewing_space() noexcept -> Metadata::ViewingSpace &;

  friend auto operator<<(std::ostream &stream, const MivSequenceParams &x) -> std::ostream &;

  auto operator==(const MivSequenceParams &other) const noexcept -> bool;
  auto operator!=(const MivSequenceParams &other) const noexcept -> bool;

  static auto decodeFrom(Metadata::InputBitstream &bitstream) -> MivSequenceParams;

  void encodeTo(Metadata::OutputBitstream &bitstream) const;

private:
  MspProfileIdc m_msp_profile_idc = MspProfileIdc::Basic;
  std::uint8_t m_msp_depth_params_num_bits = 10;
  Metadata::ViewParamsList m_view_params_list;
  bool m_msp_depth_low_quality_flag = false;

  // TODO(BK): Figure out how to signal groups in atlas data (SEI message?)
  std::size_t m_msp_num_groups = 1;

  // TODO(BK): Figure out how to signal entities in atlas data
  std::size_t m_msp_max_entities = 1;

  // TODO(BK): This could be a SEI message so it can be used by V-PCC w/o MIV
  std::optional<Metadata::ViewingSpace> m_viewing_space;

  bool m_msp_extension_present_flag = false;
};

// 23090-12 proposal: miv_parameter_set, extends 23090-5: vpcc_parameter_set
struct MivParameterSet : public VpccBitstream::VpccParameterSet {
public:
  MivParameterSet() = default;
  MivParameterSet(VpccBitstream::VpccParameterSet vps, std::optional<MivSequenceParams> mps);

  constexpr auto miv_sequence_params_present_flag() const noexcept;
  auto miv_sequence_params() const noexcept -> const MivSequenceParams &;

  auto miv_sequence_params_present_flag(bool value) noexcept -> MivParameterSet &;
  auto miv_sequence_params(MivSequenceParams value) noexcept -> MivParameterSet &;

  // When changing the view_params_list through this function, call
  // updateOverridePduProjectionIdNumBits.
  auto miv_sequence_params() noexcept -> MivSequenceParams &;

  friend auto operator<<(std::ostream &stream, const MivParameterSet &x) -> std::ostream &;

  auto operator==(const MivParameterSet &other) const noexcept -> bool;
  auto operator!=(const MivParameterSet &other) const noexcept -> bool;

  auto updateOverridePduProjectionIdNumBits() noexcept -> MivParameterSet &;
  auto pduProjectionIdNumBits() const noexcept -> unsigned;

  static auto decodeFrom(std::istream &stream, ExtensionDecoder extDecoder = noDecoderExtension)
      -> MivParameterSet;

  void encodeTo(std::ostream &stream, ExtensionEncoder extEncoder = noEncoderExtension) const;

private:
  std::optional<MivSequenceParams> m_miv_sequence_params;
};
} // namespace TMIV::MivBitstream

#include "MivParameterSet.hpp"

#endif
