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

#include <TMIV/MivBitstream/MivParameterSet.h>

#include <TMIV/Common/Common.h>

#include "verify.h"

#include <ostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, MspProfileIdc x) -> ostream & {
  switch (x) {
  case MspProfileIdc::Basic:
    return stream << "Basic";
  case MspProfileIdc::Extended:
    return stream << "Extended";
  default:
    return stream << "[unknown:" << int(x) << "]";
  }
}

auto MivSequenceParams::viewing_space() const noexcept -> const ViewingSpace & {
  VERIFY_MIVBITSTREAM(msp_viewing_space_present_flag());
  return *m_viewing_space;
}

auto MivSequenceParams::view_params_list(ViewParamsList value) -> MivSequenceParams & {
  m_view_params_list = move(value);
  return *this;
}

auto MivSequenceParams::msp_viewing_space_present_flag(bool value) noexcept -> MivSequenceParams & {
  if (value && !m_viewing_space) {
    m_viewing_space = ViewingSpace{};
  } else if (!value && m_viewing_space) {
    m_viewing_space.reset();
  }
  return *this;
}

auto MivSequenceParams::viewing_space(ViewingSpace value) noexcept -> MivSequenceParams & {
  m_viewing_space = {value};
  return *this;
}

auto MivSequenceParams::viewing_space() noexcept -> ViewingSpace & {
  VERIFY_MIVBITSTREAM(msp_viewing_space_present_flag());
  return *m_viewing_space;
}

auto operator<<(ostream &stream, const MivSequenceParams &x) -> ostream & {
  stream << "msp_profile_idc=" << x.msp_profile_idc()
         << "\nmsp_depth_params_num_bits=" << int(x.msp_depth_params_num_bits()) << '\n';
  stream << x.view_params_list();
  stream << "msp_depth_low_quality_flag=" << boolalpha << x.msp_depth_low_quality_flag()
         << "\nmsp_num_groups=" << x.msp_num_groups()
         << "\nmsp_max_entities=" << x.msp_max_entities()
         << "\nmsp_viewing_space_present_flag=" << boolalpha << x.msp_viewing_space_present_flag()
         << '\n';
  if (x.msp_viewing_space_present_flag()) {
    stream << x.viewing_space();
  }
  return stream << "msp_extension_present_flag=" << x.msp_extension_present_flag() << '\n';
}

auto MivSequenceParams::operator==(const MivSequenceParams &other) const noexcept -> bool {
  return msp_profile_idc() == other.msp_profile_idc() &&
         msp_depth_params_num_bits() == other.msp_depth_params_num_bits() &&
         view_params_list() == other.view_params_list() &&
         msp_depth_low_quality_flag() == other.msp_depth_low_quality_flag() &&
         msp_num_groups() == other.msp_num_groups() &&
         msp_max_entities() == other.msp_max_entities() &&
         m_viewing_space == other.m_viewing_space &&
         msp_extension_present_flag() == other.msp_extension_present_flag();
}

auto MivSequenceParams::operator!=(const MivSequenceParams &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivSequenceParams::decodeFrom(InputBitstream &bitstream) -> MivSequenceParams {
  auto x = MivSequenceParams{};

  x.msp_profile_idc(MspProfileIdc(bitstream.getUint8()));
  x.msp_depth_params_num_bits(uint8_t(bitstream.readBits(4) + 8));
  x.view_params_list(ViewParamsList::decodeFrom(bitstream, x.msp_depth_params_num_bits()));
  x.msp_depth_low_quality_flag(bitstream.getFlag());
  x.msp_num_groups(bitstream.getUExpGolomb() + 1);
  x.msp_max_entities(bitstream.getUExpGolomb() + 1);
  x.msp_viewing_space_present_flag(bitstream.getFlag());

  if (x.msp_viewing_space_present_flag()) {
    x.viewing_space() = ViewingSpace::decodeFrom(bitstream);
  }

  x.msp_extension_present_flag(bitstream.getFlag());

  return x;
}

void MivSequenceParams::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint8(std::uint8_t(msp_profile_idc()));

  VERIFY_MIVBITSTREAM(8 <= msp_depth_params_num_bits() && msp_depth_params_num_bits() <= 23);
  bitstream.writeBits(msp_depth_params_num_bits() - 8, 4);

  view_params_list().encodeTo(bitstream, msp_depth_params_num_bits());
  bitstream.putFlag(msp_depth_low_quality_flag());

  VERIFY_MIVBITSTREAM(1 <= msp_num_groups());
  bitstream.putUExpGolomb(msp_num_groups() - 1);

  VERIFY_MIVBITSTREAM(1 <= msp_max_entities());
  bitstream.putUExpGolomb(msp_max_entities() - 1);

  bitstream.putFlag(msp_viewing_space_present_flag());

  if (msp_viewing_space_present_flag()) {
    viewing_space().encodeTo(bitstream);
  }

  bitstream.putFlag(msp_extension_present_flag());
}

MivParameterSet::MivParameterSet(VpccParameterSet vps, optional<MivSequenceParams> mps)
    : VpccParameterSet{move(vps)}, m_miv_sequence_params{move(mps)} {}

auto MivParameterSet::miv_sequence_params_present_flag(bool value) noexcept -> MivParameterSet & {
  vps_extension_present_flag(value);
  if (value && !m_miv_sequence_params) {
    m_miv_sequence_params = MivSequenceParams{};
  } else if (!value && m_miv_sequence_params) {
    m_miv_sequence_params.reset();
  }
  return *this;
}

auto MivParameterSet::miv_sequence_params(MivSequenceParams value) noexcept -> MivParameterSet & {
  vps_extension_present_flag(true);
  m_miv_sequence_params = move(value);
  return *this;
}

auto MivParameterSet::miv_sequence_params() const noexcept -> const MivSequenceParams & {
  VERIFY_MIVBITSTREAM(miv_sequence_params_present_flag());
  return *m_miv_sequence_params;
}

auto MivParameterSet::miv_sequence_params() noexcept -> MivSequenceParams & {
  VERIFY_MIVBITSTREAM(miv_sequence_params_present_flag());
  return *m_miv_sequence_params;
}

auto operator<<(std::ostream &stream, const MivParameterSet &x) -> std::ostream & {
  stream << static_cast<const VpccParameterSet &>(x);
  stream << "miv_sequence_params_present_flag=vps_extension_present_flag\n";
  if (x.miv_sequence_params_present_flag()) {
    stream << x.miv_sequence_params();
  }
  return stream;
}

auto MivParameterSet::operator==(const MivParameterSet &other) const noexcept -> bool {
  return static_cast<const VpccParameterSet &>(*this) == other &&
         m_miv_sequence_params == other.m_miv_sequence_params;
}

auto MivParameterSet::operator!=(const MivParameterSet &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivParameterSet::updateOverridePduProjectionIdNumBits() noexcept -> MivParameterSet & {
  VpccParameterSet::overridePduProjectionIdNumBits(pduProjectionIdNumBits());
  return *this;
}

auto MivParameterSet::pduProjectionIdNumBits() const noexcept -> unsigned {
  return iceil(log2(miv_sequence_params().view_params_list().size()));
}

auto MivParameterSet::decodeFrom(std::istream &stream, ExtensionDecoder extDecoder)
    -> MivParameterSet {
  auto msp = optional<MivSequenceParams>{};
  const auto vps = VpccParameterSet::decodeFrom(stream, [&](InputBitstream &bitstream) {
    msp = MivSequenceParams::decodeFrom(bitstream);
    if (msp->msp_extension_present_flag()) {
      extDecoder(bitstream);
    }
  });

  for (auto j = 0; j < vps.vps_atlas_count(); ++j) {
    VERIFY_MIVBITSTREAM(vps.vps_map_count(j) == 1);
  }

  auto mps = MivParameterSet{vps, *msp};
  mps.updateOverridePduProjectionIdNumBits();
  return mps;
}

void MivParameterSet::encodeTo(std::ostream &stream, ExtensionEncoder extEncoder) const {
  VERIFY_MIVBITSTREAM(pduProjectionIdNumBits() ==
                      VpccParameterSet::overridePduProjectionIdNumBits());
  VpccParameterSet::encodeTo(stream, [&](OutputBitstream &bitstream) {
    miv_sequence_params().encodeTo(bitstream);
    if (miv_sequence_params().msp_extension_present_flag()) {
      extEncoder(bitstream);
    }
  });
}
} // namespace TMIV::MivBitstream