/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#ifndef TMIV_MIVBITSTREAM_CAPTUREDEVICEINFORMATION_H
#define TMIV_MIVBITSTREAM_CAPTUREDEVICEINFORMATION_H

#include "VuiParameters.h"

#include <TMIV/Common/Bitstream.h>

#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// 23090-12: capture_device_information( )
class CaptureDeviceInformation {
public:
  [[nodiscard]] auto cdi_device_model_count_minus1() const noexcept -> uint16_t;
  [[nodiscard]] auto cdi_device_model_id(uint16_t m) const -> uint8_t;
  [[nodiscard]] auto cdi_device_class_id(uint16_t m) const -> uint16_t;
  [[nodiscard]] auto cdi_sensor_count_minus1(uint16_t m) const -> uint16_t;
  [[nodiscard]] auto cdi_sensor_component_id(uint16_t m, uint16_t s) const -> uint8_t;
  [[nodiscard]] auto cdi_intra_sensor_parallax_flag(uint16_t m) const -> bool;
  [[nodiscard]] auto cdi_light_source_count(uint16_t m) const -> uint16_t;
  [[nodiscard]] auto cdi_infrared_image_present_flag(uint16_t m) const -> bool;
  [[nodiscard]] auto cdi_depth_confidence_present_flag(uint16_t m) const -> bool;
  [[nodiscard]] auto cdi_depth_confidence_flag(uint16_t m, uint16_t s) const -> bool;

  auto cdi_device_model_count_minus1(uint16_t value) -> CaptureDeviceInformation &;
  auto cdi_device_model_id(uint16_t m, uint8_t value) -> CaptureDeviceInformation &;
  auto cdi_device_class_id(uint16_t m, uint16_t value) -> CaptureDeviceInformation &;
  auto cdi_sensor_count_minus1(uint16_t m, uint16_t value) -> CaptureDeviceInformation &;
  auto cdi_sensor_component_id(uint16_t m, uint16_t s, uint8_t value) -> CaptureDeviceInformation &;
  auto cdi_intra_sensor_parallax_flag(uint16_t m, bool value) -> CaptureDeviceInformation &;
  auto cdi_light_source_count(uint16_t m, uint16_t value) -> CaptureDeviceInformation &;
  auto cdi_infrared_image_present_flag(uint16_t m, bool value) -> CaptureDeviceInformation &;
  auto cdi_depth_confidence_present_flag(uint16_t m, bool value) -> CaptureDeviceInformation &;
  auto cdi_depth_confidence_flag(uint16_t m, uint16_t s, bool value) -> CaptureDeviceInformation &;

  friend auto operator<<(std::ostream &stream, const CaptureDeviceInformation &x) -> std::ostream &;

  auto operator==(const CaptureDeviceInformation &other) const noexcept -> bool;
  auto operator!=(const CaptureDeviceInformation &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> CaptureDeviceInformation;

  void encodeTo(Common::OutputBitstream &bitstream) const;

  // helper function for the capture device information semantics
  struct Semantics {
    std::vector<uint16_t> deviceModelId{0};
    std::vector<uint16_t> deviceModelIdx{0};
    std::vector<uint16_t> deviceClassId{0};
    std::vector<uint16_t> sensorCount{0};

    std::vector<std::vector<uint8_t>> sensorComponentId{{}};
    std::vector<bool> intraSensorParallaxFlag{false};
    std::vector<uint16_t> lightSourceCount{0};
    std::vector<bool> irImagePresentFlag{false};
    std::vector<bool> depthConfidencePresentFlag{false};
    std::vector<std::vector<bool>> depthConfidenceFlag{{}};
  };

  void applySemantics(Semantics &x) const;

private:
  uint16_t m_cdi_device_model_count_minus1{0};
  std::vector<uint8_t> m_cdi_device_model_id{0};
  std::vector<uint16_t> m_cdi_device_class_id{0};
  std::vector<uint16_t> m_cdi_sensor_count_minus1{0};
  std::vector<std::vector<uint8_t>> m_cdi_sensor_component_id{{0}};
  std::vector<bool> m_cdi_intra_sensor_parallax_flag{false};
  std::vector<uint16_t> m_cdi_light_source_count{0};
  std::vector<bool> m_cdi_infrared_image_present_flag{false};
  std::vector<bool> m_cdi_depth_confidence_present_flag{false};
  std::vector<std::vector<bool>> m_cdi_depth_confidence_flag{};

  [[nodiscard]] auto checkBitstreamConstraints() const noexcept -> bool;
};
} // namespace TMIV::MivBitstream
#endif
