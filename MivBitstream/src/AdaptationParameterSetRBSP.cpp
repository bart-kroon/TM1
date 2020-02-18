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

#include <TMIV/MivBitstream/AdaptationParameterSetRBSP.h>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, const MvplUpdateMode &x) -> ostream & {
  switch (x) {
  case MvplUpdateMode::VPL_INITLIST:
    return stream << "VPL_INITLIST";
  case MvplUpdateMode::VPL_UPD_EXT:
    return stream << "VPL_UPD_EXT";
  case MvplUpdateMode::VPL_UPD_INT:
    return stream << "VPL_UPD_INT";
  case MvplUpdateMode::VPL_EXT_INT:
    return stream << "VPL_EXT_INT";
  default:
    MIVBITSTREAM_ERROR("Unknown update mode");
  }
}

auto AdaptationParameterSetRBSP::aps_miv_view_params_list_update_mode() const noexcept
    -> MvplUpdateMode {
  VERIFY_MIVBITSTREAM(m_aps_miv_view_params_list_update_mode.has_value());
  return *m_aps_miv_view_params_list_update_mode;
}

auto AdaptationParameterSetRBSP::miv_view_params_list() const noexcept
    -> const MivViewParamsList & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_list.has_value());
  return *m_miv_view_params_list;
}

auto AdaptationParameterSetRBSP::miv_view_params_update_extrinsics() const noexcept
    -> const MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_extrinsics.has_value());
  return *m_miv_view_params_update_extrinsics;
}

auto AdaptationParameterSetRBSP::miv_view_params_update_intrinsics() const noexcept
    -> const MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_intrinsics.has_value());
  return *m_miv_view_params_update_intrinsics;
}

auto operator<<(ostream &stream, const AdaptationParameterSetRBSP &x) -> ostream & {
  stream << "aps_adaptation_parameter_set_id=" << int(x.aps_adaptation_parameter_set_id())
         << "\naps_camera_params_present_flag=" << boolalpha << x.aps_camera_params_present_flag()
         << "\naps_miv_view_params_list_present_flag=" << boolalpha
         << x.aps_miv_view_params_list_present_flag() << '\n';

  if (x.aps_miv_view_params_list_present_flag()) {
    stream << "aps_miv_view_params_list_update_mode=" << x.aps_miv_view_params_list_update_mode()
           << '\n';
    stream << x.miv_view_params_list();
    stream << x.miv_view_params_update_extrinsics();
    stream << x.miv_view_params_update_intrinsics();
  }

  stream << "aps_extension2_flag=" << boolalpha << x.aps_extension2_flag() << '\n';
  return stream;
}

auto AdaptationParameterSetRBSP::decodeFrom(istream &stream) -> AdaptationParameterSetRBSP {
  InputBitstream bitstream{stream};

  auto x = AdaptationParameterSetRBSP{};

  x.aps_adaptation_parameter_set_id(uint8_t(bitstream.getUExpGolomb()));

  const auto aps_camera_params_present_flag = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(!aps_camera_params_present_flag);

  const auto aps_extension_bit_equal_to_one = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(aps_extension_bit_equal_to_one);

  x.aps_miv_view_params_list_present_flag(bitstream.getFlag());

  if (x.aps_miv_view_params_list_present_flag()) {
    x.aps_miv_view_params_list_update_mode(MvplUpdateMode(bitstream.readBits(2)));

    switch (x.aps_miv_view_params_list_update_mode()) {
    case MvplUpdateMode::VPL_INITLIST:
      x.miv_view_params_list() = MivViewParamsList::decodeFrom(bitstream);
      break;
    case MvplUpdateMode::VPL_UPD_EXT:
      x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
      break;
    case MvplUpdateMode::VPL_UPD_INT:
      x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
      break;
    case MvplUpdateMode::VPL_EXT_INT:
      x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
      x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
      break;
    }
  }
  return x;
}

void AdaptationParameterSetRBSP::encodeTo(ostream &stream) const {
  OutputBitstream bitstream{stream};

  bitstream.putUExpGolomb(aps_adaptation_parameter_set_id());

  VERIFY_MIVBITSTREAM(!aps_camera_params_present_flag());
  bitstream.putFlag(aps_camera_params_present_flag());

  constexpr auto aps_extension_bit_equal_to_one = true;
  bitstream.putFlag(aps_extension_bit_equal_to_one);

  bitstream.putFlag(aps_miv_view_params_list_present_flag());

  if (aps_miv_view_params_list_present_flag()) {
    bitstream.writeBits(uint_least64_t(aps_miv_view_params_list_update_mode()), 2);
    switch (aps_miv_view_params_list_update_mode()) {
    case MvplUpdateMode::VPL_INITLIST:
      miv_view_params_list().encodeTo(bitstream);
      break;
    case MvplUpdateMode::VPL_UPD_EXT:
      miv_view_params_update_extrinsics().encodeTo(bitstream);
      break;
    case MvplUpdateMode::VPL_UPD_INT:
      miv_view_params_update_intrinsics().encodeTo(bitstream);
      break;
    case MvplUpdateMode::VPL_EXT_INT:
      miv_view_params_update_extrinsics().encodeTo(bitstream);
      miv_view_params_update_intrinsics().encodeTo(bitstream);
      break;
    }
  }

  VERIFY_MIVBITSTREAM(!aps_extension2_flag());
  bitstream.putFlag(aps_extension2_flag());
  bitstream.rbspTrailingBits();
}
} // namespace TMIV::MivBitstream