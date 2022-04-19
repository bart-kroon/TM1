/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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
#include <TMIV/Common/verify.h>

#include <fmt/ostream.h>

#include <sstream>
#include <type_traits>
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
  case PayloadType::sei_manifest:
    return stream << "sei_manifest";
  case PayloadType::sei_prefix_indication:
    return stream << "sei_prefix_indication";
  case PayloadType::active_sub_bitstreams:
    return stream << "active_sub_bitstreams";
  case PayloadType::component_codec_mapping:
    return stream << "component_codec_mapping";
  case PayloadType::scene_object_information:
    return stream << "scene_object_information";
  case PayloadType::object_label_information:
    return stream << "object_label_information";
  case PayloadType::patch_information:
    return stream << "patch_information";
  case PayloadType::volumetric_rectangle_information:
    return stream << "volumetric_rectangle_information";
  case PayloadType::atlas_object_association:
    return stream << "atlas_object_association";
  case PayloadType::viewport_camera_parameters:
    return stream << "viewport_camera_parameters";
  case PayloadType::viewport_position:
    return stream << "viewport_position";
  case PayloadType::decoded_atlas_information_hash:
    return stream << "decoded_atlas_information_hash";
  case PayloadType::packed_independent_regions:
    return stream << "packed_independent_regions";
  case PayloadType::attribute_transformation_params:
    return stream << "attribute_transformation_params";
  case PayloadType::occupancy_synthesis:
    return stream << "occupancy_synthesis";
  case PayloadType::geometry_smoothing:
    return stream << "geometry_smoothing";
  case PayloadType::attribute_smoothing:
    return stream << "attribute_smoothing";
  case PayloadType::viewing_space:
    return stream << "viewing_space";
  case PayloadType::viewing_space_handling:
    return stream << "viewing_space_handling";
  case PayloadType::geometry_upscaling_parameters:
    return stream << "geometry_upscaling_parameters";
  case PayloadType::atlas_view_enabled:
    return stream << "atlas_view_enabled";
  case PayloadType::omaf_v1_compatible:
    return stream << "omaf_v1_compatible";
  case PayloadType::geometry_assistance:
    return stream << "geometry_assistance";
  default:
    return stream << "reserved_sei_message (" << static_cast<int32_t>(pt) << ")";
  }
}

auto operator<<(std::ostream &stream, const SeiPayload &x) -> std::ostream & {
  return std::visit(
      [&stream](const auto &payload) -> std::ostream & {
        if constexpr (std::is_same_v<decltype(payload), const std::monostate &> ||
                      std::is_same_v<decltype(payload), const std::string &>) {
          return stream; // no or unknown payload
        } else {
          return stream << payload;
        }
      },
      x.payload);
}

auto SeiPayload::operator==(const SeiPayload &other) const noexcept -> bool {
  return payload == other.payload;
}

auto SeiPayload::operator!=(const SeiPayload &other) const noexcept -> bool {
  return payload != other.payload;
}

auto SeiPayload::decodeFromString(const std::string &payload, PayloadType payloadType,
                                  NalUnitType nut) -> SeiPayload {
  std::istringstream stream{payload};
  Common::InputBitstream bitstream{stream};

  if (nut == NalUnitType::NAL_PREFIX_NSEI || nut == NalUnitType::NAL_PREFIX_ESEI) {
    switch (payloadType) {
    case PayloadType::scene_object_information:
      return {SceneObjectInformation::decodeFrom(bitstream)};
    case PayloadType::atlas_object_association:
      return {AtlasObjectAssociation::decodeFrom(bitstream)};
    case PayloadType::viewport_camera_parameters:
      return {ViewportCameraParameters::decodeFrom(bitstream)};
    case PayloadType::viewport_position:
      return {ViewportPosition::decodeFrom(bitstream)};
    case PayloadType::packed_independent_regions:
      return {PackedIndependentRegions::decodeFrom(bitstream)};
    case PayloadType::viewing_space:
      return {ViewingSpace::decodeFrom(bitstream)};
    case PayloadType::viewing_space_handling:
      return {ViewingSpaceHandling::decodeFrom(bitstream)};
    case PayloadType::geometry_upscaling_parameters:
      return {GeometryUpscalingParameters::decodeFrom(bitstream)};
    case PayloadType::atlas_view_enabled:
      return {AtlasViewEnabled::decodeFrom(bitstream)};
    case PayloadType::geometry_assistance:
      return {GeometryAssistance::decodeFrom(bitstream)};
    default:
      std::ostringstream buffer;
      buffer << stream.rdbuf();
      return {UnsupportedPayload{buffer.str()}};
    }
  } else {
    switch (payloadType) {
    case PayloadType::decoded_atlas_information_hash:
      return {DecodedAtlasInformationHash::decodeFrom(bitstream)};
    default:
      std::ostringstream buffer;
      buffer << stream.rdbuf();
      return {UnsupportedPayload{buffer.str()}};
    }
  }
}

auto SeiPayload::encodeToString(PayloadType payloadType, NalUnitType nut) const -> std::string {
  std::ostringstream stream;
  Common::OutputBitstream bitstream{stream};

  if (nut == NalUnitType::NAL_PREFIX_NSEI || nut == NalUnitType::NAL_PREFIX_ESEI) {
    switch (payloadType) {
    case PayloadType::scene_object_information:
      std::get<SceneObjectInformation>(payload).encodeTo(bitstream);
      break;
    case PayloadType::atlas_object_association:
      std::get<AtlasObjectAssociation>(payload).encodeTo(bitstream);
      break;
    case PayloadType::viewport_camera_parameters:
      std::get<ViewportCameraParameters>(payload).encodeTo(bitstream);
      break;
    case PayloadType::viewport_position:
      std::get<ViewportPosition>(payload).encodeTo(bitstream);
      break;
    case PayloadType::packed_independent_regions:
      std::get<PackedIndependentRegions>(payload).encodeTo(bitstream);
      break;
    case PayloadType::viewing_space:
      std::get<ViewingSpace>(payload).encodeTo(bitstream);
      break;
    case PayloadType::viewing_space_handling:
      std::get<ViewingSpaceHandling>(payload).encodeTo(bitstream);
      break;
    case PayloadType::geometry_upscaling_parameters:
      std::get<GeometryUpscalingParameters>(payload).encodeTo(bitstream);
      break;
    case PayloadType::atlas_view_enabled:
      std::get<AtlasViewEnabled>(payload).encodeTo(bitstream);
      break;
    case PayloadType::geometry_assistance:
      std::get<GeometryAssistance>(payload).encodeTo(bitstream);
      break;
    default:
      const auto &payload_ = std::get_if<UnsupportedPayload>(&payload);
      VERIFY(payload_ != nullptr);
      stream.write(payload_->data(), Common::assertDownCast<std::streamsize>(payload_->size()));
    }
  } else {
    const auto &payload_ = std::get_if<UnsupportedPayload>(&payload);
    VERIFY(payload_ != nullptr);
    stream.write(payload_->data(), Common::assertDownCast<std::streamsize>(payload_->size()));
  }

  if (!bitstream.byteAligned()) {
    bitstream.byteAlignment();
  }

  return stream.str();
}

SeiMessage::SeiMessage(PayloadType payloadType, SeiPayload payload)
    : m_payloadType{payloadType}, m_seiPayload{std::move(payload)} {}

auto SeiMessage::payloadType() const noexcept -> PayloadType { return m_payloadType; }

auto SeiMessage::seiPayload() const noexcept -> const SeiPayload & { return m_seiPayload; }

auto operator<<(std::ostream &stream, const SeiMessage &x) -> std::ostream & {
  stream << "payloadType=" << x.payloadType() << '\n';
  stream << x.seiPayload();
  return stream;
}

auto SeiMessage::operator==(const SeiMessage &other) const noexcept -> bool {
  return payloadType() == other.payloadType() && seiPayload() == other.seiPayload();
}

auto SeiMessage::operator!=(const SeiMessage &other) const noexcept -> bool {
  return !operator==(other);
}

auto SeiMessage::decodeFrom(std::istream &stream, NalUnitType nut) -> SeiMessage {
  auto payloadType_ = size_t{};
  auto sm_payload_type_byte = uint8_t{};

  do {
    sm_payload_type_byte = Common::getUint8(stream);
    payloadType_ += sm_payload_type_byte;
  } while (sm_payload_type_byte == 0xFF);

  const auto payloadType = static_cast<PayloadType>(payloadType_);

  auto payloadSize = size_t{};
  auto sm_payload_size_byte = uint8_t{};

  do {
    sm_payload_size_byte = Common::getUint8(stream);
    payloadSize += sm_payload_size_byte;
  } while (sm_payload_size_byte == 0xFF);

  auto payload = std::string(payloadSize, '\0');
  stream.read(payload.data(), Common::assertDownCast<std::streamsize>(payloadSize));

  return SeiMessage{payloadType, SeiPayload::decodeFromString(payload, payloadType, nut)};
}

void SeiMessage::encodeTo(std::ostream &stream, NalUnitType nut) const {
  const auto payload = seiPayload().encodeToString(payloadType(), nut);

  auto payloadType_ = static_cast<size_t>(payloadType());
  auto sm_payload_type_byte = uint8_t{};

  do {
    sm_payload_type_byte = static_cast<uint8_t>(std::min(size_t{0xFF}, payloadType_));
    Common::putUint8(stream, sm_payload_type_byte);
    payloadType_ -= sm_payload_type_byte;
  } while (sm_payload_type_byte == 0xFF);

  auto payloadSize = payload.size();
  auto sm_payload_size_byte = uint8_t{};

  do {
    sm_payload_size_byte = static_cast<uint8_t>(std::min<size_t>(0xFF, payloadSize));
    Common::putUint8(stream, sm_payload_size_byte);
    payloadSize -= sm_payload_size_byte;
  } while (sm_payload_size_byte == 0xFF);

  stream.write(payload.data(), Common::assertDownCast<std::streamsize>(payload.size()));
}

SeiRBSP::SeiRBSP(std::vector<SeiMessage> messages) : m_messages{std::move(messages)} {}

auto operator<<(std::ostream &stream, const SeiRBSP &x) -> std::ostream & {
  for (const auto &y : x.messages()) {
    stream << y;
  }
  return stream;
}

auto SeiRBSP::operator==(const SeiRBSP &other) const noexcept -> bool {
  return messages() == other.messages();
}

auto SeiRBSP::operator!=(const SeiRBSP &other) const noexcept -> bool { return !operator==(other); }

auto SeiRBSP::decodeFrom(std::istream &stream, NalUnitType nut) -> SeiRBSP {
  auto x = SeiRBSP{};

  do {
    x.messages().push_back(SeiMessage::decodeFrom(stream, nut));
  } while (Common::moreRbspData(stream));

  Common::rbspTrailingBits(stream);

  return x;
}

void SeiRBSP::encodeTo(std::ostream &stream, NalUnitType nut) const {
  PRECONDITION(!messages().empty());

  for (const auto &x : messages()) {
    x.encodeTo(stream, nut);
  }
  Common::rbspTrailingBits(stream);
}
} // namespace TMIV::MivBitstream
