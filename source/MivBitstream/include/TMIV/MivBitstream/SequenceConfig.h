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

#ifndef TMIV_MIVBITSTREAM_SEQUENCECONFIG_H
#define TMIV_MIVBITSTREAM_SEQUENCECONFIG_H

#include "ViewParamsList.h"

#include <TMIV/Common/Frame.h>

#include <variant>

namespace TMIV::MivBitstream {
struct CameraConfig {
  ViewParams viewParams;

  uint32_t bitDepthGeometry{};
  uint32_t bitDepthTexture{};
  uint32_t bitDepthTransparency{};
  uint32_t bitDepthEntities{};

  Common::ColorFormat colorFormatGeometry{Common::ColorFormat::YUV420};
  Common::ColorFormat colorFormatTexture{Common::ColorFormat::YUV420};
  Common::ColorFormat colorFormatTransparency{Common::ColorFormat::YUV420};
  Common::ColorFormat colorFormatEntities{Common::ColorFormat::YUV420};

  CameraConfig() = default;
  explicit CameraConfig(const Common::Json &config);

  explicit operator Common::Json() const;

  auto operator==(const CameraConfig &other) const noexcept -> bool;
  auto operator!=(const CameraConfig &other) const noexcept -> bool;
};

struct SequenceConfig {
  Common::Vec3d boundingBoxCenter;
  std::string contentName;
  double frameRate{};
  int32_t numberOfFrames{};
  std::vector<CameraConfig> cameras;
  std::vector<std::string> sourceCameraNames;
  std::vector<uint16_t> sourceCameraIds;
  bool lengthsInMeters{true};

  SequenceConfig() = default;
  explicit SequenceConfig(const Common::Json &config);
  explicit SequenceConfig(std::istream &stream);

  explicit operator Common::Json() const;

  [[nodiscard]] auto cameraByName(const std::string &name) const -> CameraConfig;
  [[nodiscard]] auto sourceViewParams() const -> ViewParamsList;

  auto operator==(const SequenceConfig &other) const noexcept -> bool;
  auto operator!=(const SequenceConfig &other) const noexcept -> bool;
};
} // namespace TMIV::MivBitstream

#endif
