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

#include <TMIV/MivBitstream/SequenceConfig.h>

#include <TMIV/Common/Math.h>
#include <TMIV/Common/Quaternion.h>

#include <regex>

using namespace std::string_literals;

namespace TMIV::MivBitstream {
CameraConfig::CameraConfig(const Common::Json &config) {
  bitDepthColor = config.require("BitDepthColor").as<int>();
  bitDepthDepth = config.require("BitDepthDepth").as<int>();
  viewParams = ViewParams{config};
}

CameraConfig::operator Common::Json() const {
  using Common::Json;
  using Object = Json::Object;

  auto root = Json{viewParams}.as<Object>();
  root["BitDepthColor"s] = bitDepthColor;
  root["BitDepthDepth"s] = bitDepthDepth;
  root["ColorSpace"] = "YUV420";
  root["DepthColorSpace"] = "YUV420";

  return Json{root};
}

auto CameraConfig::operator==(const CameraConfig &other) const noexcept -> bool {
  return viewParams == other.viewParams && bitDepthColor == other.bitDepthColor &&
         bitDepthDepth == other.bitDepthDepth && colorspace == other.colorspace &&
         depthColorspace == other.depthColorspace;
}

auto CameraConfig::operator!=(const CameraConfig &other) const noexcept -> bool {
  return !operator==(other);
}

SequenceConfig::SequenceConfig(const Common::Json &config) {
  boundingBoxCenter = config.require("BoundingBox_center").asVec<double, 3>();
  contentName = config.require("Content_name").as<std::string>();
  frameRate = config.require("Fps").as<double>();
  numberOfFrames = config.require("Frames_number").as<int>();

  if (const auto &node = config.optional("sourceCameraNames")) {
    sourceCameraNames = node.asVector<std::string>();
  }

  const auto &node = config.require("cameras").as<Common::Json::Array>();

  cameras.resize(node.size());
  std::transform(node.cbegin(), node.cend(), cameras.begin(),
                 [](const Common::Json &node) { return CameraConfig{node}; });

  if (sourceCameraNames.empty()) {
    const auto pattern = std::regex{"v[0-9]+"};

    for (const auto &camera : cameras) {
      if (std::regex_match(camera.viewParams.name, pattern)) {
        sourceCameraNames.push_back(camera.viewParams.name);
      }
    }
  }
}

SequenceConfig::SequenceConfig(std::istream &stream)
    : SequenceConfig{Common::Json::loadFrom(stream)} {}

SequenceConfig::operator Common::Json() const {
  using Common::Json;
  using Object = Json::Object;
  using Array = Json::Array;

  Object root;
  root["BoundingBox_center"s] =
      Array{Json{boundingBoxCenter.x()}, Json{boundingBoxCenter.y()}, Json{boundingBoxCenter.z()}};
  root["Content_name"s] = contentName;
  root["Fps"] = frameRate;
  root["Frames_number"] = numberOfFrames;

  if (!sourceCameraNames.empty()) {
    auto a = Array{};
    for (const auto &name : sourceCameraNames) {
      a.emplace_back(Json{name});
    }
    root["sourceCameraNames"] = std::move(a);
  }

  auto a = Array{};
  for (const auto &camera : cameras) {
    a.push_back(Json{camera});
  }
  root["cameras"] = std::move(a);

  return Json{root};
}

auto SequenceConfig::sourceViewParams() const -> ViewParamsList {
  auto vpl = std::vector<ViewParams>{};

  for (const auto &camera : cameras) {
    if (Common::contains(sourceCameraNames, camera.viewParams.name)) {
      vpl.push_back(camera.viewParams);
    }
  }

  if (vpl.size() != sourceCameraNames.size()) {
    throw std::runtime_error("Sequence config: Could not find all source cameras by name");
  }

  return ViewParamsList{vpl};
}

auto SequenceConfig::operator==(const SequenceConfig &other) const noexcept -> bool {
  return boundingBoxCenter == other.boundingBoxCenter && contentName == other.contentName &&
         frameRate == other.frameRate && numberOfFrames == other.numberOfFrames &&
         cameras == other.cameras && sourceCameraNames == other.sourceCameraNames;
}

auto SequenceConfig::operator!=(const SequenceConfig &other) const noexcept -> bool {
  return !operator==(other);
}
} // namespace TMIV::MivBitstream
