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

#ifndef TMIV_MIVBITSTREAM_V3CSAMPLESTREAMFORMAT_H
#define TMIV_MIVBITSTREAM_V3CSAMPLESTREAMFORMAT_H

#include <cstdint>
#include <iosfwd>
#include <string>

namespace TMIV::MivBitstream {
// 23090-5: sample_stream_v3c_header()
class SampleStreamV3cHeader {
public:
  explicit SampleStreamV3cHeader(uint8_t ssvh_unit_size_precision_bytes_minus1);

  [[nodiscard]] constexpr auto ssvh_unit_size_precision_bytes_minus1() const noexcept {
    return m_ssvh_unit_size_precision_bytes_minus1;
  }

  friend auto operator<<(std::ostream &stream, const SampleStreamV3cHeader &x) -> std::ostream &;

  constexpr auto operator==(const SampleStreamV3cHeader &other) const noexcept -> bool {
    return ssvh_unit_size_precision_bytes_minus1() == other.ssvh_unit_size_precision_bytes_minus1();
  }

  constexpr auto operator!=(const SampleStreamV3cHeader &other) const noexcept -> bool {
    return !operator==(other);
  }

  static auto decodeFrom(std::istream &stream) -> SampleStreamV3cHeader;

  void encodeTo(std::ostream &stream) const;

private:
  uint8_t m_ssvh_unit_size_precision_bytes_minus1{};
};

// 23090-5: sample_stream_v3c_unit()
class SampleStreamV3cUnit {
public:
  explicit SampleStreamV3cUnit(std::string ssvu_v3c_unit);

  [[nodiscard]] auto ssvu_v3c_unit_size() const noexcept { return m_ssvu_v3c_unit.size(); }
  [[nodiscard]] auto ssvu_v3c_unit() const noexcept -> const auto & { return m_ssvu_v3c_unit; }

  friend auto operator<<(std::ostream &stream, const SampleStreamV3cUnit &x) -> std::ostream &;

  auto operator==(const SampleStreamV3cUnit &other) const noexcept -> bool;
  auto operator!=(const SampleStreamV3cUnit &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, const SampleStreamV3cHeader &header)
      -> SampleStreamV3cUnit;

  void encodeTo(std::ostream &stream, const SampleStreamV3cHeader &header) const;

private:
  std::string m_ssvu_v3c_unit;
};
} // namespace TMIV::MivBitstream

#endif
