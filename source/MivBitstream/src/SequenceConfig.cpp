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

#include <TMIV/MivBitstream/SequenceConfig.h>

#include <TMIV/Common/Math.h>
#include <TMIV/Common/Quaternion.h>
#include <TMIV/Common/format.h>

#include <regex>

using namespace std::string_literals;

namespace TMIV::MivBitstream {
CameraConfig::CameraConfig(const Common::Json &config) {
  if (const auto &node = config.optional("BitDepthColor")) {
    bitDepthTexture = node.as<uint32_t>();
  }
  if (const auto &node = config.optional("BitDepthTransparency")) {
    bitDepthTransparency = node.as<uint32_t>();
  }
  if (const auto &node = config.optional("BitDepthDepth")) {
    bitDepthGeometry = node.as<uint32_t>();
  }
  if (const auto &node = config.optional("BitDepthEntities")) {
    bitDepthEntities = node.as<uint32_t>();
  }
  viewParams = ViewParams{config};
}

CameraConfig::operator Common::Json() const {
  using Common::Json;
  using Object = Json::Object;

  auto root = Json{viewParams}.as<Object>();

  if (0 < bitDepthTexture) {
    root["BitDepthColor"s] = bitDepthTexture;
    root["ColorSpace"] = "YUV420";
  }

  if (0 < bitDepthGeometry) {
    root["BitDepthDepth"s] = bitDepthGeometry;
    root["DepthColorSpace"] = "YUV420";
  }

  if (0 < bitDepthTransparency) {
    root["BitDepthTransparency"s] = bitDepthTransparency;
    root["TransparencyColorSpace"] = "YUV420";
  }

  if (0 < bitDepthEntities) {
    root["BitDepthEntities"s] = bitDepthEntities;
    root["EntitiesColorSpace"] = "YUV420";
  }
  return Json{root};
}

auto CameraConfig::operator==(const CameraConfig &other) const noexcept -> bool {
  return viewParams == other.viewParams && bitDepthTexture == other.bitDepthTexture &&
         bitDepthTransparency == other.bitDepthTransparency &&
         bitDepthGeometry == other.bitDepthGeometry && bitDepthEntities == other.bitDepthEntities &&
         colorFormatTexture == other.colorFormatTexture &&
         colorFormatGeometry == other.colorFormatGeometry &&
         colorFormatTransparency == other.colorFormatTransparency &&
         colorFormatEntities == other.colorFormatEntities;
}

auto CameraConfig::operator!=(const CameraConfig &other) const noexcept -> bool {
  return !operator==(other);
}

namespace {
void checkVersion(const std::string &text) {
  if (!std::regex_match(text, std::regex{"[1-9][0-9]*\\.[0-9]+"})) {
    throw std::runtime_error(
        "The Version field in the sequence configuration has to be of the form major.minor.");
  }

  const auto version = std::stof(text);

  if (4.F <= version && version < 5.F) {
    return; // Supported version
  }
  if (version < 4.F) {
    throw std::runtime_error("There is no support for previous versions of the sequence "
                             "configuration format due to inconsistent use of the Version key.");
  }
  throw std::runtime_error(
      "Future major versions of the sequence configuration format do not have to be backwards "
      "compatible. Refusing to read this configuration file.");
}
} // namespace

SequenceConfig::SequenceConfig(const Common::Json &config) {
  checkVersion(config.require("Version").as<std::string>());

  boundingBoxCenter = config.require("BoundingBox_center").asVec<double, 3>();
  contentName = config.require("Content_name").as<std::string>();
  frameRate = config.require("Fps").as<double>();
  numberOfFrames = config.require("Frames_number").as<int32_t>();

  if (const auto &node = config.optional("sourceCameraNames")) {
    sourceCameraNames = node.asVector<std::string>();
  }

  if (const auto &node = config.optional("sourceCameraIds")) {
    sourceCameraIds = node.asVector<uint16_t>();
  }

  {
    const auto &node = config.require("cameras").as<Common::Json::Array>();

    cameras.resize(node.size());
    std::transform(node.cbegin(), node.cend(), cameras.begin(),
                   [](const Common::Json &node_) { return CameraConfig{node_}; });
  }

  if (sourceCameraNames.empty()) {
    const auto pattern = std::regex{"v[0-9]+"};

    for (const auto &camera : cameras) {
      if (std::regex_match(camera.viewParams.name, pattern)) {
        sourceCameraNames.push_back(camera.viewParams.name);
      }
    }
  }
  lengthsInMeters = config.require("lengthsInMeters").as<bool>();
}

SequenceConfig::SequenceConfig(std::istream &stream)
    : SequenceConfig{Common::Json::loadFrom(stream)} {}

SequenceConfig::operator Common::Json() const {
  using Common::Json;
  using Object = Json::Object;
  using Array = Json::Array;

  Object root;
  root["Version"] = "4.0"s;
  root["BoundingBox_center"s] =
      Array{Json{boundingBoxCenter.x()}, Json{boundingBoxCenter.y()}, Json{boundingBoxCenter.z()}};
  root["Content_name"s] = contentName;
  root["Fps"] = frameRate;
  root["Frames_number"] = numberOfFrames;
  root["lengthsInMeters"] = lengthsInMeters;

  if (!sourceCameraNames.empty()) {
    auto a = Array{};
    for (const auto &name : sourceCameraNames) {
      a.emplace_back(Json{name});
    }
    root["sourceCameraNames"] = std::move(a);
  }

  if (!sourceCameraIds.empty()) {
    auto a = Array{};
    for (const auto &cameraId : sourceCameraIds) {
      a.emplace_back(Json{cameraId});
    }
    root["sourceCameraIds"] = std::move(a);
  }

  auto a = Array{};
  for (const auto &camera : cameras) {
    a.push_back(Json{camera});
  }
  root["cameras"] = std::move(a);

  return Json{root};
}

[[nodiscard]] auto SequenceConfig::cameraByName(const std::string &name) const -> CameraConfig {
  auto i = std::find_if(cameras.cbegin(), cameras.cend(), [&name](const CameraConfig &camera) {
    return camera.viewParams.name == name;
  });
  if (i == cameras.cend()) {
    throw std::runtime_error(
        TMIV_FMT::format("There is no camera named {} in the sequence configuration", name));
  }
  return *i;
}

auto SequenceConfig::sourceViewParams() const -> ViewParamsList {
  auto vpl = ViewParamsList{};
  std::transform(sourceCameraNames.cbegin(), sourceCameraNames.cend(), std::back_inserter(vpl),
                 [this](const std::string &name) { return cameraByName(name).viewParams; });
  vpl.assignViewIds(sourceCameraIds);
  vpl.constructViewIdIndex();
  return ViewParamsList{vpl};
}

auto SequenceConfig::operator==(const SequenceConfig &other) const noexcept -> bool {
  return boundingBoxCenter == other.boundingBoxCenter && contentName == other.contentName &&
         frameRate == other.frameRate && numberOfFrames == other.numberOfFrames &&
         cameras == other.cameras && sourceCameraNames == other.sourceCameraNames &&
         sourceCameraIds == other.sourceCameraIds;
}

auto SequenceConfig::operator!=(const SequenceConfig &other) const noexcept -> bool {
  return !operator==(other);
}
} // namespace TMIV::MivBitstream
