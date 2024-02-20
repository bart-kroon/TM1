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

#ifndef TMIV_MIVBITSTREAM_SEIRBSP_H
#define TMIV_MIVBITSTREAM_SEIRBSP_H

#include <TMIV/Common/Bitstream.h>

#include <TMIV/MivBitstream/AtlasObjectAssociation.h>
#include <TMIV/MivBitstream/AtlasViewEnabled.h>
#include <TMIV/MivBitstream/DecodedAtlasInformationHash.h>
#include <TMIV/MivBitstream/ExtendedGeometryAssistance.h>
#include <TMIV/MivBitstream/GeometryAssistance.h>
#include <TMIV/MivBitstream/GeometryUpscalingParameters.h>
#include <TMIV/MivBitstream/PackedIndependentRegions.h>
#include <TMIV/MivBitstream/SceneObjectInformation.h>
#include <TMIV/MivBitstream/ViewingSpace.h>
#include <TMIV/MivBitstream/ViewingSpaceHandling.h>
#include <TMIV/MivBitstream/ViewportCameraParameters.h>
#include <TMIV/MivBitstream/ViewportPosition.h>

#include <variant>
#include <vector>

namespace TMIV::MivBitstream {
enum class PayloadType : uint16_t {
  buffering_period,
  atlas_frame_timing,
  filler_payload,
  user_data_registered_itu_t_t35,
  user_data_unregistered,
  recovery_point,
  no_reconstruction,
  time_code,
  sei_manifest,
  sei_prefix_indication,
  active_sub_bitstreams,
  component_codec_mapping,
  scene_object_information,
  object_label_information,
  patch_information,
  volumetric_rectangle_information,
  atlas_object_association,
  viewport_camera_parameters,
  viewport_position,
  decoded_atlas_information_hash,
  packed_independent_regions,
  attribute_transformation_params = 64, // V-PCC
  occupancy_synthesis,
  geometry_smoothing,
  attribute_smoothing,
  vpcc_registered_sei_message,
  viewing_space = 128, // MIV
  viewing_space_handling,
  geometry_upscaling_parameters,
  atlas_view_enabled,
  omaf_v1_compatible,
  geometry_assistance,
  extended_geometry_assistance,
  miv_registered_sei_message
};

auto operator<<(std::ostream &stream, PayloadType pt) -> std::ostream &;

enum class MivPayloadType : uint16_t {};

auto operator<<(std::ostream &stream, MivPayloadType mpt) -> std::ostream &;

// 23090-12: miv_registered_sei_payload( mivPayloadType, mivPayloadSize )
struct MivRegisteredSeiPayload {
  using UnsupportedPayload = std::string;

  using Payload = std::variant<std::monostate, UnsupportedPayload>;

  Payload payload;

  friend auto operator<<(std::ostream &stream, const MivRegisteredSeiPayload &x) -> std::ostream &;

  auto operator==(const MivRegisteredSeiPayload &other) const noexcept -> bool;
  auto operator!=(const MivRegisteredSeiPayload &other) const noexcept -> bool;

  static auto decodeFromString(const std::string &payload, MivPayloadType payloadType)
      -> MivRegisteredSeiPayload;

  [[nodiscard]] auto encodeToString(MivPayloadType payloadType) const -> std::string;
};

// 23090-12: miv_registered_sei_message( payloadSize )
class MivRegisteredSeiMessage {
public:
  MivRegisteredSeiMessage() = default;
  MivRegisteredSeiMessage(MivPayloadType mivPayloadType, MivRegisteredSeiPayload payload);

  [[nodiscard]] auto mivPayloadType() const noexcept -> MivPayloadType;
  [[nodiscard]] auto mivRegisteredSeiPayload() const noexcept -> const MivRegisteredSeiPayload &;

  friend auto operator<<(std::ostream &stream, const MivRegisteredSeiMessage &x) -> std::ostream &;

  auto operator==(const MivRegisteredSeiMessage &other) const noexcept -> bool;
  auto operator!=(const MivRegisteredSeiMessage &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> MivRegisteredSeiMessage;

  void encodeTo(std::ostream &stream) const;

private:
  MivPayloadType m_mivPayloadType{};
  MivRegisteredSeiPayload m_mivRegisteredSeiPayload;
};

// 23090-5: sei_payload( payloadType, payloadSize )
struct SeiPayload {
  using UnsupportedPayload = std::string;

  using Payload = std::variant<
      std::monostate, UnsupportedPayload, SceneObjectInformation, AtlasObjectAssociation,
      ViewportCameraParameters, ViewportPosition, PackedIndependentRegions, ViewingSpace,
      ViewingSpaceHandling, GeometryUpscalingParameters, AtlasViewEnabled, GeometryAssistance,
      ExtendedGeometryAssistance, DecodedAtlasInformationHash, MivRegisteredSeiMessage>;

  Payload payload;

  friend auto operator<<(std::ostream &stream, const SeiPayload &x) -> std::ostream &;

  auto operator==(const SeiPayload &other) const noexcept -> bool;
  auto operator!=(const SeiPayload &other) const noexcept -> bool;

  static auto decodeFromString(const std::string &payload, PayloadType payloadType, NalUnitType nut)
      -> SeiPayload;

  [[nodiscard]] auto encodeToString(PayloadType payloadType, NalUnitType nut) const -> std::string;
};

// 23090-5: sei_message()
class SeiMessage {
public:
  SeiMessage() = default;
  SeiMessage(PayloadType payloadType, SeiPayload payload);

  [[nodiscard]] auto payloadType() const noexcept -> PayloadType;
  [[nodiscard]] auto seiPayload() const noexcept -> const SeiPayload &;

  friend auto operator<<(std::ostream &stream, const SeiMessage &x) -> std::ostream &;

  auto operator==(const SeiMessage &other) const noexcept -> bool;
  auto operator!=(const SeiMessage &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, NalUnitType nut) -> SeiMessage;

  void encodeTo(std::ostream &stream, NalUnitType nut) const;

private:
  PayloadType m_payloadType{};
  SeiPayload m_seiPayload;
};

// 23090-5: sei_rbsp()
class SeiRBSP {
public:
  SeiRBSP() = default;
  explicit SeiRBSP(std::vector<SeiMessage> messages);

  [[nodiscard]] constexpr auto messages() const noexcept -> const auto & { return m_messages; }
  constexpr auto messages() noexcept -> auto & { return m_messages; }

  friend auto operator<<(std::ostream &stream, const SeiRBSP &x) -> std::ostream &;

  auto operator==(const SeiRBSP &other) const noexcept -> bool;
  auto operator!=(const SeiRBSP &other) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream, NalUnitType nut) -> SeiRBSP;

  void encodeTo(std::ostream &stream, NalUnitType nut) const;

private:
  std::vector<SeiMessage> m_messages;
};
} // namespace TMIV::MivBitstream

#endif
