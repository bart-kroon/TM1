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

#include <TMIV/Decoder/CommonAtlasDecoder.h>

#include <TMIV/MivBitstream/verify.h>

#include "NalUnitSemantics.h"

#include <cassert>
#include <sstream>
#include <utility>

namespace TMIV::Decoder {
CommonAtlasDecoder::CommonAtlasDecoder(V3cUnitSource source, MivBitstream::V3cParameterSet vps,
                                       int32_t foc)
    : m_source{std::move(source)}, m_vps{std::move(vps)}, m_foc{foc} {}

auto CommonAtlasDecoder::operator()() -> std::optional<AccessUnit> {
  if (!m_buffer.empty() || decodeAsb()) {
    return decodeAu();
  }
  return {};
}

auto CommonAtlasDecoder::decodeAsb() -> bool {
  if (auto asb = m_source()) {
    for (const auto &nu : asb->v3c_payload().atlas_sub_bitstream().nal_units()) {
      if (nu.nal_unit_header().nal_layer_id() == 0) {
        m_buffer.push_back(nu);
      } else {
        std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
      }
    }
    VERIFY_V3CBITSTREAM(!m_buffer.empty());
    return true;
  }
  return false;
}

auto CommonAtlasDecoder::decodeAu() -> AccessUnit {
  auto au = AccessUnit{};

  const auto nut = [this]() { return m_buffer.front().nal_unit_header().nal_unit_type(); };

  if (!m_buffer.empty() && isAud(nut())) {
    m_buffer.pop_front();
  }

  while (!m_buffer.empty() && isPrefixNalUnit(nut())) {
    decodePrefixNalUnit(au, m_buffer.front());
    m_buffer.pop_front();
  }

  VERIFY_V3CBITSTREAM(!m_buffer.empty() && isCaf(nut()));
  decodeCafNalUnit(au, m_buffer.front());
  m_buffer.pop_front();

  while (!m_buffer.empty() && isSuffixNalUnit(nut())) {
    decodeSuffixNalUnit(m_buffer.front());
    m_buffer.pop_front();
  }

  if (!m_buffer.empty() && isEos(nut())) {
    m_buffer.pop_front();
  }

  if (!m_buffer.empty() && isEob(nut())) {
    m_buffer.pop_front();
  }

  const auto focLsb = au.caf.caf_frm_order_cnt_lsb();
  VERIFY_V3CBITSTREAM(focLsb < m_maxFrmOrderCntLsb);
  while (++m_foc % m_maxFrmOrderCntLsb != focLsb) {
    // deliberately empty
  }
  std::cout << "Common atlas frame: foc=" << m_foc << '\n';
  au.foc = m_foc;

  return au;
}

void CommonAtlasDecoder::decodePrefixNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu) {
  std::istringstream stream{nu.rbsp()};

  switch (nu.nal_unit_header().nal_unit_type()) {
  case MivBitstream::NalUnitType::NAL_AAPS:
    return decodeAaps(stream);
  case MivBitstream::NalUnitType::NAL_PREFIX_NSEI:
    return decodeSei(au.prefixNSei_gup, stream);
  default:
    std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
  }
}

void CommonAtlasDecoder::decodeCafNalUnit(AccessUnit &au, const MivBitstream::NalUnit &nu) {
  std::istringstream stream{nu.rbsp()};

  VERIFY_MIVBITSTREAM(0 < m_maxFrmOrderCntLsb);
  au.caf = MivBitstream::CommonAtlasFrameRBSP::decodeFrom(stream, m_vps, m_maxFrmOrderCntLsb);
  au.aaps = aapsById(m_aapsV, au.caf.caf_atlas_adaptation_parameter_set_id());
}

void CommonAtlasDecoder::decodeSuffixNalUnit(const MivBitstream::NalUnit &nu) {
  std::istringstream stream{nu.rbsp()};

  if (nu.nal_unit_header().nal_unit_type() == MivBitstream::NalUnitType::NAL_FD) {
    return;
  }
  std::cout << "WARNING: Ignoring NAL unit:\n" << nu;
}

void CommonAtlasDecoder::decodeAaps(std::istream &stream) {
  auto aaps = MivBitstream::AtlasAdaptationParameterSetRBSP::decodeFrom(stream);

  if (aaps.aaps_log2_max_afoc_present_flag()) {
    const auto x = 1U << (aaps.aaps_log2_max_atlas_frame_order_cnt_lsb_minus4() + 4U);
    VERIFY_MIVBITSTREAM(m_maxFrmOrderCntLsb == 0 || m_maxFrmOrderCntLsb == x);
    m_maxFrmOrderCntLsb = x;
  }

  for (auto &x : m_aapsV) {
    if (x.aaps_atlas_adaptation_parameter_set_id() ==
        aaps.aaps_atlas_adaptation_parameter_set_id()) {
      x = std::move(aaps);
      return;
    }
  }
  return m_aapsV.push_back(aaps);
}

void CommonAtlasDecoder::decodeSei(MivBitstream::SeiMessage &gup_message, std::istream &stream) {
  auto sei_rbsp = MivBitstream::SeiRBSP::decodeFrom(stream);
  for (auto &message : sei_rbsp.messages()) {
    if (message.payloadType() == MivBitstream::PayloadType::geometry_upscaling_parameters) {
      gup_message = message;
    }
  }
}
} // namespace TMIV::Decoder
