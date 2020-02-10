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

#include <TMIV/VpccBitstream/VpccSampleStreamFormat.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Bytestream.h>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::VpccBitstream {
SampleStreamVpccHeader::SampleStreamVpccHeader(int ssvh_unit_size_precision_bytes)
    : m_ssvh_unit_size_precision_bytes{uint8_t(ssvh_unit_size_precision_bytes)} {
  VERIFY_VPCCBITSTREAM(0 < ssvh_unit_size_precision_bytes && ssvh_unit_size_precision_bytes <= 8);
}

auto operator<<(ostream &stream, const SampleStreamVpccHeader &x) -> ostream & {
  return stream << "ssvh_unit_size_precision_bytes=" << int(x.ssvh_unit_size_precision_bytes())
                << '\n';
}

auto SampleStreamVpccHeader::decodeFrom(istream &stream) -> SampleStreamVpccHeader {
  InputBitstream bitstream{stream};
  const auto ssvh_unit_size_precision_bytes_minus1 = bitstream.readBits(3);
  return SampleStreamVpccHeader{int(ssvh_unit_size_precision_bytes_minus1 + 1)};
}

void SampleStreamVpccHeader::encodeTo(ostream &stream) const {
  OutputBitstream bitstream{stream};
  bitstream.writeBits(m_ssvh_unit_size_precision_bytes - 1, 3);
}

SampleStreamVpccUnit::SampleStreamVpccUnit(string ssvu_vpcc_unit)
    : m_ssvu_vpcc_unit{move(ssvu_vpcc_unit)} {}

auto operator<<(ostream &stream, const SampleStreamVpccUnit &x) -> ostream & {
  return stream << "vpcc_unit(" << x.ssvu_vpcc_unit_size() << ")\n";
}

auto SampleStreamVpccUnit::operator==(const SampleStreamVpccUnit &other) const noexcept -> bool {
  return m_ssvu_vpcc_unit == other.m_ssvu_vpcc_unit;
}

auto SampleStreamVpccUnit::operator!=(const SampleStreamVpccUnit &other) const noexcept -> bool {
  return !operator==(other);
}

auto SampleStreamVpccUnit::decodeFrom(istream &stream, const SampleStreamVpccHeader &header)
    -> SampleStreamVpccUnit {
  const auto ssvu_vpcc_unit_size = readBytes(stream, header.ssvh_unit_size_precision_bytes());
  return SampleStreamVpccUnit{readString(stream, size_t(ssvu_vpcc_unit_size))};
}

void SampleStreamVpccUnit::encodeTo(ostream &stream, const SampleStreamVpccHeader &header) const {
  writeBytes(stream, m_ssvu_vpcc_unit.size(), header.ssvh_unit_size_precision_bytes());
  stream.write(m_ssvu_vpcc_unit.data(), m_ssvu_vpcc_unit.size());
}
} // namespace TMIV::VpccBitstream
