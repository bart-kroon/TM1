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

#ifndef _TMIV_MIVBITSTREAM_MIVVUIPARAMS_H_
#define _TMIV_MIVBITSTREAM_MIVVUIPARAMS_H_

#include <TMIV/Common/Bitstream.h>

namespace TMIV::MivBitstream {
// 23090-12: coordinate_axis_system_params()
class CoordinateAxisSystemParams {
public:
  constexpr auto cas_forward_axis() const noexcept;
  constexpr auto cas_delta_left_axis_minus1() const noexcept;
  constexpr auto cas_forward_sign() const noexcept;
  constexpr auto cas_left_sign() const noexcept;
  constexpr auto cas_up_sign() const noexcept;

  constexpr auto &cas_forward_axis(std::uint8_t value) noexcept;
  constexpr auto &cas_delta_left_axis_minus1(std::uint8_t value) noexcept;
  constexpr auto &cas_forward_sign(bool value) noexcept;
  constexpr auto &cas_left_sign(bool value) noexcept;
  constexpr auto &cas_up_sign(bool value) noexcept;

  constexpr auto isOmafCas() const noexcept;

  friend auto operator<<(std::ostream &stream, const CoordinateAxisSystemParams &x)
      -> std::ostream &;

  constexpr auto operator==(const CoordinateAxisSystemParams &other) const noexcept;
  constexpr auto operator!=(const CoordinateAxisSystemParams &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> CoordinateAxisSystemParams;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  std::uint8_t m_cas_forward_axis{};
  std::uint8_t m_cas_delta_left_axis_minus1{};
  bool m_cas_forward_sign{true};
  bool m_cas_left_sign{true};
  bool m_cas_up_sign{true};
};

// 23090-12: miv_vui_params()
class MivVuiParams {
public:
  constexpr MivVuiParams() = default;
  explicit constexpr MivVuiParams(const CoordinateAxisSystemParams &cas) : m_cas{cas} {}

  constexpr auto &coordinate_axis_system_params() const noexcept;
  constexpr auto &coordinate_axis_system_params() noexcept;

  friend auto operator<<(std::ostream &stream, const MivVuiParams &x) -> std::ostream &;

  constexpr auto operator==(const MivVuiParams &other) const noexcept;
  constexpr auto operator!=(const MivVuiParams &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MivVuiParams;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  CoordinateAxisSystemParams m_cas;
};
} // namespace TMIV::MivBitstream

#include "MivVuiParams.hpp"

#endif
