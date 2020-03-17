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

#include <TMIV/MivBitstream/NalUnit.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>

#include "verify.h"

#include <algorithm>
#include <utility>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, NalUnitType x) -> ostream & {
  switch (x) {
  case NalUnitType::NAL_TRAIL:
    return stream << "NAL_TRAIL";
  case NalUnitType::NAL_TSA:
    return stream << "NAL_TSA";
  case NalUnitType::NAL_STSA:
    return stream << "NAL_STSA";
  case NalUnitType::NAL_RADL:
    return stream << "NAL_RADL";
  case NalUnitType::NAL_RASL:
    return stream << "NAL_RASL";
  case NalUnitType::NAL_SKIP:
    return stream << "NAL_SKIP";
  case NalUnitType::NAL_BLA_W_LP:
    return stream << "NAL_BLA_W_LP";
  case NalUnitType::NAL_BLA_W_RADL:
    return stream << "NAL_BLA_W_RADL";
  case NalUnitType::NAL_BLA_N_LP:
    return stream << "NAL_BLA_N_LP";
  case NalUnitType::NAL_GBLA_W_LP:
    return stream << "NAL_GBLA_W_LP";
  case NalUnitType::NAL_GBLA_W_RADL:
    return stream << "NAL_GBLA_W_RADL";
  case NalUnitType::NAL_GBLA_N_LP:
    return stream << "NAL_GBLA_N_LP";
  case NalUnitType::NAL_IDR_W_RADL:
    return stream << "NAL_IDR_W_RADL";
  case NalUnitType::NAL_IDR_N_LP:
    return stream << "NAL_IDR_N_LP";
  case NalUnitType::NAL_GIDR_W_RADL:
    return stream << "NAL_GIDR_W_RADL";
  case NalUnitType::NAL_GIDR_N_LP:
    return stream << "NAL_GIDR_N_LP";
  case NalUnitType::NAL_CRA:
    return stream << "NAL_CRA";
  case NalUnitType::NAL_GCRA:
    return stream << "NAL_GCRA";
  case NalUnitType::NAL_ASPS:
    return stream << "NAL_ASPS";
  case NalUnitType::NAL_AFPS:
    return stream << "NAL_AFPS";
  case NalUnitType::NAL_AUD:
    return stream << "NAL_AUD";
  case NalUnitType::NAL_VPCC_AUD:
    return stream << "NAL_VPCC_AUD";
  case NalUnitType::NAL_EOS:
    return stream << "NAL_EOS";
  case NalUnitType::NAL_EOB:
    return stream << "NAL_EOB";
  case NalUnitType::NAL_FD:
    return stream << "NAL_FD";
  case NalUnitType::NAL_PREFIX_NSEI:
    return stream << "NAL_PREFIX_NSEI";
  case NalUnitType::NAL_SUFFIX_NSEI:
    return stream << "NAL_SUFFIX_NSEI";
  case NalUnitType::NAL_PREFIX_ESEI:
    return stream << "NAL_PREFIX_ESEI";
  case NalUnitType::NAL_SUFFIX_ESEI:
    return stream << "NAL_SUFFIX_ESEI";
  case NalUnitType::NAL_APS:
    return stream << "NAL_APS";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

NalUnitHeader::NalUnitHeader(NalUnitType nal_unit_type, int nal_layer_id, int nal_temporal_id_plus1)
    : m_nal_unit_type{nal_unit_type}, m_nal_layer_id{uint8_t(nal_layer_id)},
      m_nal_temporal_id_plus1{uint8_t(nal_temporal_id_plus1)} {
  VERIFY_VPCCBITSTREAM(0 <= nal_layer_id && nal_layer_id <= 63);
  VERIFY_VPCCBITSTREAM(0 < nal_temporal_id_plus1 && nal_temporal_id_plus1 <= 7);
}

auto operator<<(ostream &stream, const NalUnitHeader &x) -> ostream & {
  return stream << "nal_unit_type=" << x.m_nal_unit_type
                << "\nnal_layer_id=" << int(x.m_nal_layer_id)
                << "\nnal_temporal_id_plus1=" << int(x.m_nal_temporal_id_plus1) << '\n';
}

auto NalUnitHeader::decodeFrom(istream &stream) -> NalUnitHeader {
  InputBitstream bitstream{stream};
  const auto nal_forbidden_zero_bit = bitstream.getFlag();
  VERIFY_VPCCBITSTREAM(!nal_forbidden_zero_bit);
  const auto nal_unit_type = bitstream.readBits<NalUnitType>(6);
  const auto nal_layer_id = bitstream.readBits<int>(6);
  const auto nal_temporal_id_plus1 = bitstream.readBits<int>(3);
  VERIFY_VPCCBITSTREAM(nal_temporal_id_plus1 > 0);
  return NalUnitHeader{nal_unit_type, nal_layer_id, nal_temporal_id_plus1};
}

void NalUnitHeader::encodeTo(ostream &stream) const {
  OutputBitstream bitstream{stream};
  bitstream.putFlag(false);
  bitstream.writeBits(m_nal_unit_type, 6);
  bitstream.writeBits(m_nal_layer_id, 6);
  bitstream.writeBits(m_nal_temporal_id_plus1, 3);
}

NalUnit::NalUnit(const NalUnitHeader &nal_unit_header, string rbsp)
    : m_nal_unit_header{nal_unit_header}, m_rbsp{std::move(std::move(rbsp))} {}

auto operator<<(ostream &stream, const NalUnit &x) -> ostream & {
  return stream << x.m_nal_unit_header << "NumBytesInRbsp=" << x.m_rbsp.size() << "\n";
}

auto NalUnit::decodeFrom(istream &stream, size_t numBytesInNalUnit) -> NalUnit {
  const auto nal_unit_header = NalUnitHeader::decodeFrom(stream);
  if (numBytesInNalUnit == 2) {
    return NalUnit{nal_unit_header, {}};
  }
  auto rbsp = readString(stream, numBytesInNalUnit - 2);
  return NalUnit{nal_unit_header, rbsp};
}

auto NalUnit::encodeTo(std::ostream &stream) const -> std::size_t {
  m_nal_unit_header.encodeTo(stream);
  stream.write(m_rbsp.data(), m_rbsp.size());
  return size();
}
} // namespace TMIV::MivBitstream
