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

#include <TMIV/Common/verify.h>
#include <TMIV/MivBitstream/CaptureDeviceInformation.h>

#include <fmt/ostream.h>
#include <set>

namespace TMIV::MivBitstream {
auto CaptureDeviceInformation::cdi_device_model_count_minus1() const noexcept -> uint16_t {
  return m_cdi_device_model_count_minus1;
}

auto CaptureDeviceInformation::cdi_device_model_count_minus1(uint16_t value)
    -> CaptureDeviceInformation & {
  m_cdi_device_model_count_minus1 = value;

  auto sz = m_cdi_device_model_id.size();
  m_cdi_device_model_id.resize(value + 1);
  for (auto m = sz; m < m_cdi_device_model_id.size(); m++) {
    m_cdi_device_model_id[m] = static_cast<uint8_t>(m);
  }
  m_cdi_device_class_id.resize(value + 1);
  m_cdi_sensor_count_minus1.resize(value + 1);
  m_cdi_sensor_component_id.resize(value + 1);
  m_cdi_intra_sensor_parallax_flag.resize(value + 1);
  m_cdi_light_source_count.resize(value + 1);
  m_cdi_infrared_image_present_flag.resize(value + 1);
  m_cdi_depth_confidence_present_flag.resize(value + 1);
  m_cdi_depth_confidence_flag.resize(value + 1);
  return *this;
}

auto CaptureDeviceInformation::cdi_device_model_id(uint16_t m) const -> uint8_t {
  VERIFY(m < m_cdi_device_model_id.size());
  return m_cdi_device_model_id[m];
}

auto CaptureDeviceInformation::cdi_device_class_id(uint16_t m) const -> uint16_t {
  VERIFY(m < m_cdi_device_class_id.size());
  return m_cdi_device_class_id[m];
}

auto CaptureDeviceInformation::cdi_sensor_count_minus1(uint16_t m) const -> uint16_t {
  VERIFY(m < m_cdi_device_class_id.size());
  return m_cdi_device_class_id[m];
}

auto CaptureDeviceInformation::cdi_sensor_component_id(uint16_t m, uint16_t s) const -> uint8_t {
  VERIFY(m < m_cdi_sensor_component_id.size());
  VERIFY(s < m_cdi_sensor_component_id[m].size());
  return m_cdi_sensor_component_id[m][s];
}

auto CaptureDeviceInformation::cdi_intra_sensor_parallax_flag(uint16_t m) const -> bool {
  VERIFY(m < m_cdi_intra_sensor_parallax_flag.size());
  return m_cdi_intra_sensor_parallax_flag[m];
}

auto CaptureDeviceInformation::cdi_light_source_count(uint16_t m) const -> uint16_t {
  VERIFY(m < m_cdi_light_source_count.size());
  return m_cdi_light_source_count[m];
}

auto CaptureDeviceInformation::cdi_infrared_image_present_flag(uint16_t m) const -> bool {
  VERIFY(m < m_cdi_infrared_image_present_flag.size());
  return m_cdi_infrared_image_present_flag[m];
}

auto CaptureDeviceInformation::cdi_depth_confidence_present_flag(uint16_t m) const -> bool {
  VERIFY(m < m_cdi_depth_confidence_present_flag.size());
  return m_cdi_depth_confidence_present_flag[m];
}

auto CaptureDeviceInformation::cdi_depth_confidence_flag(uint16_t m, uint16_t s) const -> bool {
  VERIFY(m < m_cdi_depth_confidence_flag.size());
  VERIFY(s < m_cdi_depth_confidence_flag[m].size());
  return m_cdi_depth_confidence_flag[m][s];
}

auto CaptureDeviceInformation::cdi_device_model_id(uint16_t m, uint8_t value)
    -> CaptureDeviceInformation & {
  VERIFY(m < m_cdi_device_model_id.size());
  m_cdi_device_model_id[m] = value;
  return *this;
}

auto CaptureDeviceInformation::cdi_device_class_id(uint16_t m, uint16_t value)
    -> CaptureDeviceInformation & {
  VERIFY(m < m_cdi_device_class_id.size());
  m_cdi_device_class_id[m] = value;
  return *this;
}

auto CaptureDeviceInformation::cdi_sensor_count_minus1(uint16_t m, uint16_t value)
    -> CaptureDeviceInformation & {
  VERIFY(m < m_cdi_sensor_count_minus1.size());
  m_cdi_sensor_count_minus1[m] = value;

  m_cdi_sensor_component_id[m].resize(value + 1);
  m_cdi_depth_confidence_flag[m].resize(value + 1);
  return *this;
}

auto CaptureDeviceInformation::cdi_sensor_component_id(uint16_t m, uint16_t s, uint8_t value)
    -> CaptureDeviceInformation & {
  VERIFY(m < m_cdi_sensor_component_id.size());
  VERIFY(s < m_cdi_sensor_component_id[m].size());
  m_cdi_sensor_component_id[m][s] = value;
  return *this;
}

auto CaptureDeviceInformation::cdi_intra_sensor_parallax_flag(uint16_t m, bool value)
    -> CaptureDeviceInformation & {
  VERIFY(m < m_cdi_intra_sensor_parallax_flag.size());
  m_cdi_intra_sensor_parallax_flag[m] = value;
  return *this;
}

auto CaptureDeviceInformation::cdi_light_source_count(uint16_t m, uint16_t value)
    -> CaptureDeviceInformation & {
  VERIFY(m < m_cdi_light_source_count.size());
  m_cdi_light_source_count[m] = value;
  return *this;
}

auto CaptureDeviceInformation::cdi_infrared_image_present_flag(uint16_t m, bool value)
    -> CaptureDeviceInformation & {
  VERIFY(m < m_cdi_infrared_image_present_flag.size());
  m_cdi_infrared_image_present_flag[m] = value;
  return *this;
}

auto CaptureDeviceInformation::cdi_depth_confidence_present_flag(uint16_t m, bool value)
    -> CaptureDeviceInformation & {
  VERIFY(m < m_cdi_depth_confidence_present_flag.size());
  m_cdi_depth_confidence_present_flag[m] = value;
  return *this;
}

auto CaptureDeviceInformation::cdi_depth_confidence_flag(uint16_t m, uint16_t s, bool value)
    -> CaptureDeviceInformation & {
  VERIFY(m < m_cdi_depth_confidence_flag.size());
  VERIFY(s < m_cdi_depth_confidence_flag[m].size());
  m_cdi_depth_confidence_flag[m][s] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const CaptureDeviceInformation &x) -> std::ostream & {
  fmt::print(stream, "cdi_device_model_count_minus1={}\n", x.cdi_device_model_count_minus1());
  for (uint16_t m = 0; m <= x.cdi_device_model_count_minus1(); m++) {
    fmt::print(stream, "cdi_device_model_id[{}]={}\n", m, x.cdi_device_model_id(m));
    fmt::print(stream, "cdi_device_class_id[{}]={}\n", m, x.cdi_device_class_id(m));
    if (x.cdi_device_class_id(m) != 0) {
      fmt::print(stream, "cdi_sensor_count_minus1[{}]={}\n", m, x.cdi_sensor_count_minus1(m));
      for (uint16_t s = 0; s <= x.cdi_sensor_count_minus1(m); s++) {
        fmt::print(stream, "cdi_sensor_component_id[{}][{}]={}\n", m, s,
                   x.cdi_sensor_component_id(m, s));
      }

      fmt::print(stream, "cdi_intra_sensor_parallax_flag[{}]={}\n", m,
                 x.cdi_intra_sensor_parallax_flag(m));
      fmt::print(stream, "cdi_light_source_count[{}]={}\n", m, x.cdi_light_source_count(m));
      fmt::print(stream, "cdi_infrared_image_present_flag[{}]={}\n", m,
                 x.cdi_infrared_image_present_flag(m));
      fmt::print(stream, "cdi_depth_confidence_present_flag[{}]={}\n", m,
                 x.cdi_depth_confidence_present_flag(m));

      if (x.cdi_depth_confidence_present_flag(m)) {
        for (uint16_t s = 0; s <= x.cdi_sensor_count_minus1(m); s++) {
          if (x.cdi_sensor_component_id(m, s) == 0) {
            fmt::print(stream, "cdi_depth_confidence_flag[{}][{}]={}\n", m, s,
                       x.cdi_depth_confidence_flag(m, s));
          }
        }
      }
    }
  }

  return stream;
}

auto CaptureDeviceInformation::operator==(const CaptureDeviceInformation &other) const noexcept
    -> bool {
  if (cdi_device_model_count_minus1() != other.cdi_device_model_count_minus1()) {
    return false;
  }

  for (uint16_t m = 0; m <= cdi_device_model_count_minus1(); m++) {
    if ((cdi_device_model_id(m) != other.cdi_device_model_id(m)) ||
        (cdi_device_class_id(m) != other.cdi_device_class_id(m))) {
      return false;
    }

    if (cdi_device_class_id(m) != 0) {
      if (cdi_sensor_count_minus1(m) != other.cdi_sensor_count_minus1(m) ||
          m_cdi_sensor_component_id[m] != other.m_cdi_sensor_component_id[m] ||
          cdi_intra_sensor_parallax_flag(m) != other.cdi_intra_sensor_parallax_flag(m) ||
          cdi_light_source_count(m) != other.cdi_light_source_count(m) ||
          cdi_infrared_image_present_flag(m) != other.cdi_infrared_image_present_flag(m) ||
          cdi_depth_confidence_present_flag(m) != other.cdi_depth_confidence_present_flag(m)) {
        return false;
      }
      if (cdi_depth_confidence_present_flag(m) &&
          (m_cdi_depth_confidence_flag[m] != other.m_cdi_depth_confidence_flag[m])) {
        return false;
      }
    }
  }
  return true;
}

auto CaptureDeviceInformation::operator!=(const CaptureDeviceInformation &other) const noexcept
    -> bool {
  return !operator==(other);
}

auto CaptureDeviceInformation::decodeFrom(Common::InputBitstream &bitstream)
    -> CaptureDeviceInformation {
  auto x = CaptureDeviceInformation{};

  x.cdi_device_model_count_minus1(bitstream.getUExpGolomb<uint16_t>());
  for (uint16_t m = 0; m <= x.cdi_device_model_count_minus1(); m++) {
    x.cdi_device_model_id(m, bitstream.readBits<uint8_t>(6));
    x.cdi_device_class_id(m, bitstream.getUExpGolomb<uint16_t>());
    if (x.cdi_device_class_id(m) != 0) {
      x.cdi_sensor_count_minus1(m, bitstream.getUExpGolomb<uint16_t>());
      for (uint16_t s = 0; s <= x.cdi_sensor_count_minus1(m); s++) {
        x.cdi_sensor_component_id(m, s, bitstream.readBits<uint8_t>(5));
      }

      x.cdi_intra_sensor_parallax_flag(m, bitstream.getFlag());
      x.cdi_light_source_count(m, bitstream.getUExpGolomb<uint16_t>());
      x.cdi_infrared_image_present_flag(m, bitstream.getFlag());
      x.cdi_depth_confidence_present_flag(m, bitstream.getFlag());

      if (x.cdi_depth_confidence_present_flag(m)) {
        for (uint16_t s = 0; s <= x.cdi_sensor_count_minus1(m); s++) {
          if (x.cdi_sensor_component_id(m, s) == 0) {
            x.cdi_depth_confidence_flag(m, s, bitstream.getFlag());
          }
        }
      }
    }
  }

  VERIFY_MIVBITSTREAM(x.checkBitstreamConstraints());
  return x;
}

void CaptureDeviceInformation::encodeTo(Common::OutputBitstream &bitstream) const {
  VERIFY_MIVBITSTREAM(checkBitstreamConstraints());
  bitstream.putUExpGolomb(cdi_device_model_count_minus1());
  for (uint16_t m = 0; m <= cdi_device_model_count_minus1(); m++) {
    bitstream.writeBits(cdi_device_model_id(m), 6);
    bitstream.putUExpGolomb(cdi_device_class_id(m));
    if (cdi_device_class_id(m) != 0) {
      bitstream.putUExpGolomb(cdi_sensor_count_minus1(m));
      for (uint16_t s = 0; s <= cdi_sensor_count_minus1(m); s++) {
        bitstream.writeBits(cdi_sensor_component_id(m, s), 5);
      }

      bitstream.putFlag(cdi_intra_sensor_parallax_flag(m));
      bitstream.putUExpGolomb(cdi_light_source_count(m));
      bitstream.putFlag(cdi_infrared_image_present_flag(m));
      bitstream.putFlag(cdi_depth_confidence_present_flag(m));

      if (cdi_depth_confidence_present_flag(m)) {
        for (uint16_t s = 0; s <= cdi_sensor_count_minus1(m); s++) {
          if (cdi_sensor_component_id(m, s) == 0) {
            bitstream.putFlag(cdi_depth_confidence_flag(m, s));
          }
        }
      }
    }
  }
}

auto CaptureDeviceInformation::checkBitstreamConstraints() const noexcept -> bool {
  // The value of cdi_device_model_id[ m ] shall be in the range 0 to 63, inclusive.
  for (uint8_t id : m_cdi_device_model_id) {
    if (id > 63) {
      return false;
    }
  }

  // cdi_device_model_id[ m ] shall not be equal to cdi_device_model_id[ n ] for all m != n.
  std::set<uint16_t> unique_values(m_cdi_device_model_id.begin(), m_cdi_device_model_id.end());
  return (unique_values.size() == m_cdi_device_model_id.size());
}

void CaptureDeviceInformation::applySemantics(Semantics &x) const {
  uint16_t deviceModelCount = cdi_device_model_count_minus1() + 1;
  x.deviceModelId.resize(deviceModelCount, 0);
  for (uint16_t m = 0; m < deviceModelCount; m++) {
    auto i = cdi_device_model_id(m);
    x.deviceModelId[m] = i;

    if (i >= x.deviceModelIdx.size()) {
      x.deviceModelIdx.resize(i + 1);
      x.deviceClassId.resize(i + 1);
    }
    x.deviceModelIdx[i] = m;
    x.deviceClassId[i] = cdi_device_class_id(m);
  }

  for (uint16_t m = 0; m < deviceModelCount; m++) {
    auto i = x.deviceModelId[m];

    if (i >= x.sensorCount.size()) {
      x.sensorCount.resize(i + 1);
      x.intraSensorParallaxFlag.resize(i + 1);
      x.lightSourceCount.resize(i + 1);
      x.irImagePresentFlag.resize(i + 1);
      x.depthConfidencePresentFlag.resize(i + 1);
      x.depthConfidenceFlag.resize(i + 1);
      x.sensorComponentId.resize(i + 1);
    }
    if (x.deviceClassId[i] == 0) {
      x.sensorCount[i] = 0;
    } else {
      x.sensorCount[i] = cdi_sensor_count_minus1(m) + 1;
    }

    x.intraSensorParallaxFlag[i] = cdi_intra_sensor_parallax_flag(m);
    x.lightSourceCount[i] = cdi_light_source_count(m);
    x.irImagePresentFlag[i] = cdi_infrared_image_present_flag(m);
    x.depthConfidencePresentFlag[i] = cdi_depth_confidence_present_flag(m);
    for (uint16_t s = 0; s < x.sensorCount[i]; s++) {
      if (s >= x.sensorComponentId[i].size()) {
        x.sensorComponentId[i].resize(s + 1);
      }
      x.sensorComponentId[i][s] = cdi_sensor_component_id(m, s);
      if (cdi_depth_confidence_present_flag(m)) {
        if (s >= x.depthConfidenceFlag[i].size()) {
          x.depthConfidenceFlag[i].resize(s + 1);
        }
        x.depthConfidenceFlag[i][s] = cdi_depth_confidence_flag(m, s);
      }
    }
  }
}

} // namespace TMIV::MivBitstream
