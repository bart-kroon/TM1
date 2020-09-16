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

#include <TMIV/MivBitstream/SeiRBSP.h>

#include <TMIV/Common/Bytestream.h>
#include <TMIV/MivBitstream/MivDecoderMode.h>
#include <TMIV/MivBitstream/verify.h>

#include <utility>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, PayloadType pt) -> std::ostream & {
  switch (pt) {
  case PayloadType::buffering_period:
    return stream << "buffering_period";
  case PayloadType::atlas_frame_timing:
    return stream << "atlas_frame_timing";
  case PayloadType::filler_payload:
    return stream << "filler_payload";
  case PayloadType::user_data_registered_itu_t_t35:
    return stream << "user_data_registered_itu_t_t35";
  case PayloadType::user_data_unregistered:
    return stream << "user_data_unregistered";
  case PayloadType::recovery_point:
    return stream << "recovery_point";
  case PayloadType::no_display:
    return stream << "no_display";
  case PayloadType::time_code:
    return stream << "time_code";
  case PayloadType::regional_nesting:
    return stream << "regional_nesting";
  case PayloadType::sei_manifest:
    return stream << "sei_manifest";
  case PayloadType::sei_prefix_indication:
    return stream << "sei_prefix_indication";
  case PayloadType::geometry_transformation_params:
    return stream << "geometry_transformation_params";
  case PayloadType::attribute_transformation_params:
    return stream << "attribute_transformation_params";
  case PayloadType::active_sub_bitstreams:
    return stream << "active_sub_bitstreams";
  case PayloadType::component_codec_mapping:
    return stream << "component_codec_mapping";
  case PayloadType::volumetric_tiling_info:
    return stream << "volumetric_tiling_info";
  case PayloadType::presentation_information:
    return stream << "presentation_information";
  case PayloadType::geometry_smoothing:
    return stream << "geometry_smoothing";
  case PayloadType::attribute_smoothing:
    return stream << "attribute_smoothing";
  case PayloadType::viewing_space:
    return stream << "viewing_space";
  case PayloadType::rec_viewport:
    return stream << "rec_viewport";
  case PayloadType::viewing_space_handling:
    return stream << "viewing_space_handling";
  case PayloadType::geometry_upscaling_parameters:
    return stream << "geometry_upscaling_parameters";
  default:
    return stream << "reserved_sei_message (" << static_cast<int>(pt) << ")";
  }
}

SeiMessage::SeiMessage(PayloadType pt, std::string payload)
    : m_payloadType{pt}, m_payload{std::move(std::move(payload))} {}

auto SeiMessage::payloadType() const noexcept -> PayloadType { return m_payloadType; }

auto SeiMessage::payloadSize() const noexcept -> size_t { return payload().size(); }

auto SeiMessage::payload() const noexcept -> const std::string & { return m_payload; }

auto operator<<(std::ostream &stream, const SeiMessage &x) -> std::ostream & {
  stream << "payloadType=" << x.payloadType() << '\n';
  stream << "payloadSize=" << x.payloadSize() << '\n';
  return stream;
}

auto SeiMessage::operator==(const SeiMessage &other) const noexcept -> bool {
  return payloadType() == other.payloadType() && payloadSize() == other.payloadSize() &&
         payload() == other.payload();
}

auto SeiMessage::operator!=(const SeiMessage &other) const noexcept -> bool {
  return !operator==(other);
}

namespace {
auto decodeSeiHeaderValue(std::istream &stream) -> size_t {
  size_t value = 0;
  uint8_t sm_payload_type_byte = 0;
  do {
    sm_payload_type_byte = Common::getUint8(stream);
    value += sm_payload_type_byte;
  } while (sm_payload_type_byte == UINT8_MAX);
  return value;
}
} // namespace

auto SeiMessage::decodeFrom(std::istream &stream) -> SeiMessage {
  const auto payloadType = PayloadType(decodeSeiHeaderValue(stream));
  const auto payloadSize = decodeSeiHeaderValue(stream);
  auto buffer = std::vector<char>(payloadSize);
  stream.read(buffer.data(), buffer.size());
  return {payloadType, std::string(buffer.data(), buffer.size())};
}

namespace {
void encodeSeiHeaderValue(std::ostream &stream, size_t value) {
  while (value >= UINT8_MAX) {
    Common::putUint8(stream, UINT8_MAX);
    value -= UINT8_MAX;
  }
  Common::putUint8(stream, uint8_t(value));
}
} // namespace

void SeiMessage::encodeTo(std::ostream &stream) const {
  encodeSeiHeaderValue(stream, unsigned(payloadType()));
  encodeSeiHeaderValue(stream, payloadSize());
  stream.write(payload().data(), payload().size());
}

SeiRBSP::SeiRBSP(std::vector<SeiMessage> messages) : m_messages{std::move(messages)} {}

auto operator<<(std::ostream &stream, const SeiRBSP &x) -> std::ostream & {
  for (const auto &x : x.messages()) {
    stream << x;
  }
  return stream;
}

auto SeiRBSP::operator==(const SeiRBSP &other) const noexcept -> bool {
  return messages() == other.messages();
}

auto SeiRBSP::operator!=(const SeiRBSP &other) const noexcept -> bool { return !operator==(other); }

auto SeiRBSP::decodeFrom(std::istream &stream) -> SeiRBSP {
  auto messages = std::vector<SeiMessage>{};

  do {
    messages.push_back(SeiMessage::decodeFrom(stream));
  } while (Common::moreRbspData(stream));
  if (mode != MivDecoderMode::TMC2) {
    Common::rbspTrailingBits(stream);
  }

  return SeiRBSP{messages};
}

void SeiRBSP::encodeTo(std::ostream &stream) const {
  VERIFY_MIVBITSTREAM(!messages().empty());

  for (const auto &x : messages()) {
    x.encodeTo(stream);
  }
  Common::rbspTrailingBits(stream);
}
} // namespace TMIV::MivBitstream
