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

#include <TMIV/VpccBitstream/AtlasSubBitstream.h>

#include "verify.h"

using namespace std;

namespace TMIV::VpccBitstream {
const auto &AtlasSubBitstream::nal_sample_stream() const noexcept {
  VERIFY_MIVBITSTREAM(!!m_nss);
  return *m_nss;
}

auto operator<<(ostream &stream, const AtlasSubBitstream &x) -> ostream & {
  stream << x.nal_sample_stream();
  for (const auto &asps : x.atlas_sequence_parameter_sets()) {
    stream << asps;
  }
  return stream;
}

auto AtlasSubBitstream::operator==(const AtlasSubBitstream &other) const noexcept -> bool {
  return nal_sample_stream() == other.nal_sample_stream();
}

auto AtlasSubBitstream::operator!=(const AtlasSubBitstream &other) const noexcept -> bool {
  return !operator==(other);
}

auto AtlasSubBitstream::decodeFrom(istream &stream) -> AtlasSubBitstream {
  auto asb = AtlasSubBitstream{NalSampleStream::decodeFrom(stream)};
  for (const auto &nal_unit : asb.nal_sample_stream().nal_units()) {
    asb.decodeNalUnit(nal_unit);
  }
  return asb;
}

void AtlasSubBitstream::encodeTo(ostream &stream) const { nal_sample_stream().encodeTo(stream); }

void AtlasSubBitstream::decodeNalUnit(const NalUnit &nal_unit) {
  if (nal_unit.nal_unit_header().nal_unit_type() == NalUnitType::NAL_ASPS) {
    return decodeAsps(nal_unit);
  }
  if (nal_unit.nal_unit_header().nal_unit_type() == NalUnitType::NAL_AFPS) {
    return decodeAfps(nal_unit);
  }
}

void AtlasSubBitstream::decodeAsps(const NalUnit &nal_unit) {
  istringstream substream{nal_unit.rbsp()};
  auto asps = AtlasSequenceParameterSetRBSP::decodeFrom(substream);
  while (asps.asps_atlas_sequence_parameter_set_id() >= m_asps.size()) {
    m_asps.emplace_back();
  }
  m_asps[asps.asps_atlas_sequence_parameter_set_id()] = move(asps);
}

void AtlasSubBitstream::decodeAfps(const NalUnit &nal_unit) {
  istringstream substream{nal_unit.rbsp()};
  auto afps = AtlasFrameParameterSetRBSP::decodeFrom(substream, m_asps);
  while (afps.afps_atlas_frame_parameter_set_id() >= m_afps.size()) {
    m_afps.emplace_back();
  }
  m_afps[afps.afps_atlas_frame_parameter_set_id()] = move(afps);
}
} // namespace TMIV::VpccBitstream
