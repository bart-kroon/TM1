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

#include <TMIV/IO/IO.h>

#include <TMIV/MivBitstream/SequenceConfig.h>

#include <cassert>
#include <functional>
#include <iomanip>
#include <iostream>
#include <regex>

namespace TMIV::IO {
namespace {
// The TMIV encoder always loads texture (and may use it internally) but attribute video data (AVD)
// is an optional output in MIV WD4.
auto haveTexture(const Common::Json &config) { return !config.optional("noTexture"); }

// check if explicit occupancy coding mode
auto explicitOccupancy(const Common::Json &config) {
  return config.require("explicitOccupancy").as<bool>();
}
} // namespace

auto loadSourceParams(const Common::Json &config) -> MivBitstream::EncoderParams {
  auto x = MivBitstream::EncoderParams{haveTexture(config), explicitOccupancy(config)};

  std::string viewPath = getFullPath(config, "SourceDirectory", "SourceCameraParameters");

  std::ifstream stream{viewPath};
  if (!stream.good()) {
    throw std::runtime_error("Failed to load source camera parameters\n" + viewPath);
  }

  auto sequenceConfig = MivBitstream::SequenceConfig{stream};

  if (const auto &node = config.optional("SourceCameraNames")) {
    std::cout << "WARNING: Source camera names are derived from the sequence configuration. This "
                 "functionality to override source camera names is only for internal testing, e.g. "
                 "to test with a subset of views.\n";
    sequenceConfig.sourceCameraNames = node.asVector<std::string>();
  }

  x.viewParamsList = sequenceConfig.sourceViewParams();
  x.frameRate = sequenceConfig.frameRate;

  if (const auto &node = config.optional("depthLowQualityFlag")) {
    x.vme().vme_depth_low_quality_flag(node.as<bool>());
  }

  const auto numGroups = static_cast<unsigned>(config.require("numGroups").as<int>());
  if (numGroups < 1) {
    throw std::runtime_error("Require numGroups >= 1");
  }
  x.vme().vme_num_groups_minus1(numGroups - 1U);

  const auto maxEntities = static_cast<unsigned>(config.require("maxEntities").as<int>());
  if (maxEntities < 1) {
    throw std::runtime_error("Require maxEntities >= 1");
  }
  x.vme().vme_max_entities_minus1(maxEntities - 1U);

  if (const auto &subnode = config.optional("ViewingSpace")) {
    x.viewingSpace = MivBitstream::ViewingSpace::loadFromJson(subnode, config);
  }

  if (config.require("OmafV1CompatibleFlag").as<bool>()) {
    x.aaps.aaps_miv_extension_present_flag(true).aaps_miv_extension().aame_omaf_v1_compatible_flag(
        true);
  }
  return x;
}

namespace {
auto loadSourceTexture(const Common::Json &config, const Common::Vec2i &size,
                       const std::string &viewName, int frameIndex) {
  auto frame = readFrame<Common::YUV420P10>(config, "SourceDirectory", "SourceTexturePathFmt",
                                            frameIndex, size, viewName);
  if (frame.empty()) {
    throw std::runtime_error("Failed to read source texture frame");
  }
  return frame;
}

template <typename FORMAT>
auto loadSourceDepth_(int bits, const Common::Json &config, const Common::Vec2i &size,
                      const std::string &viewName, int frameIndex) {
  auto depth16 = Common::Depth16Frame{size.x(), size.y()};

  const auto depth = readFrame<FORMAT>(config, "SourceDirectory", "SourceGeometryPathFmt",
                                       frameIndex, size, viewName);
  if (depth.empty()) {
    throw std::runtime_error("Failed to read source geometry frame");
  }

  std::transform(std::begin(depth.getPlane(0)), std::end(depth.getPlane(0)),
                 std::begin(depth16.getPlane(0)), [bits](unsigned x) {
                   const auto x_max = Common::maxLevel(bits);
                   assert(0 <= x && x <= x_max);
                   const auto y = (0xFFFF * x + x_max / 2) / x_max;
                   assert(0 <= y && y <= UINT16_MAX);
                   return static_cast<uint16_t>(y);
                 });

  return depth16;
}

auto loadSourceDepth(const Common::Json &config, const Common::Vec2i &size,
                     const std::string &viewName, int frameIndex) {
  const auto bits = config.require("SourceGeometryBitDepth").as<int>();

  if (0 < bits && bits <= 8) {
    return loadSourceDepth_<Common::YUV400P8>(bits, config, size, viewName, frameIndex);
  }
  if (8 < bits && bits <= 16) {
    return loadSourceDepth_<Common::YUV400P16>(bits, config, size, viewName, frameIndex);
  }
  throw std::runtime_error("Invalid SourceGeometryBitDepth");
}

template <typename FORMAT>
auto loadSourceEntities_(const Common::Json &config, const Common::Vec2i size,
                         const std::string &viewName, int frameIndex) {
  auto entities16 = Common::EntityMap{size.x(), size.y()};

  const auto entities = readFrame<FORMAT>(config, "SourceDirectory", "SourceEntityPathFmt",
                                          frameIndex, size, viewName);
  if (entities.empty()) {
    throw std::runtime_error("Failed to read source entities frame");
  }

  std::copy(entities.getPlane(0).begin(), entities.getPlane(0).end(),
            entities16.getPlane(0).begin());

  return entities16;
}

auto loadSourceEntities(const Common::Json &config, const Common::Vec2i size,
                        const std::string &viewName, int frameIndex) {
  if (const auto &node = config.optional("SourceEntityBitDepth")) {
    const auto bits = node.as<int>();
    if (0 < bits && bits <= 8) {
      return loadSourceEntities_<Common::YUV400P8>(config, size, viewName, frameIndex);
    }
    if (8 < bits && bits <= 16) {
      return loadSourceEntities_<Common::YUV400P16>(config, size, viewName, frameIndex);
    }
    throw std::runtime_error("Invalid SourceEntityBitDepth");
  }
  return Common::EntityMap{};
}
} // namespace

auto loadSourceFrame(const Common::Json &config, const Common::SizeVector &sizes, int frameIndex)
    -> Common::MVD16Frame {
  auto frame = Common::MVD16Frame(sizes.size());

  frameIndex += config.require("startFrame").as<int>();

  const auto viewNames = config.require("SourceCameraNames").asVector<std::string>();
  assert(viewNames.size() == sizes.size());

  for (size_t viewId = 0; viewId < frame.size(); ++viewId) {
    auto &view = frame[viewId];
    view.texture = loadSourceTexture(config, sizes[viewId], viewNames[viewId], frameIndex);
    view.depth = loadSourceDepth(config, sizes[viewId], viewNames[viewId], frameIndex);
    view.entities = loadSourceEntities(config, sizes[viewId], viewNames[viewId], frameIndex);
  }

  return frame;
}

void saveAtlas(const Common::Json &config, int frameIndex, const Common::MVD10Frame &frame) {
  for (size_t atlasId = 0; atlasId < frame.size(); ++atlasId) {
    if (haveTexture(config)) {
      writeFrame(config, "AttributeVideoDataPathFmt", frame[atlasId].texture, frameIndex, "T",
                 static_cast<int>(atlasId));
    }
    writeFrame(config, "GeometryVideoDataPathFmt", frame[atlasId].depth, frameIndex,
               static_cast<int>(atlasId));
    if (!frame[atlasId].occupancy.empty()) {
      writeFrame(config, "OccupancyVideoDataPathFmt", frame[atlasId].occupancy, frameIndex,
                 static_cast<int>(atlasId));
    }
  }
}

void saveBlockToPatchMaps(const Common::Json &config, int frameIndex,
                          const Decoder::AccessUnit &frame) {
  for (size_t atlasId = 0; atlasId < frame.atlas.size(); ++atlasId) {
    writeFrame(config, "AtlasPatchOccupancyMapFmt", frame.atlas[atlasId].blockToPatchMap,
               frameIndex, static_cast<int>(atlasId));
  }
}

void savePrunedFrame(const Common::Json &config, int frameIndex,
                     const std::pair<std::vector<Common::Texture444Depth10Frame>, Common::MaskList>
                         &prunedViewsAndMasks) {
  for (size_t viewId = 0; viewId < prunedViewsAndMasks.first.size(); ++viewId) {
    const auto &view = prunedViewsAndMasks.first[viewId];
    if (config.optional("PrunedViewAttributePathFmt")) {
      IO::writeFrame(config, "PrunedViewAttributePathFmt", view.first, frameIndex, "T", viewId);
    }
    if (config.optional("PrunedViewGeometryPathFmt")) {
      IO::writeFrame(config, "PrunedViewGeometryPathFmt", view.second, frameIndex, viewId);
    }
    if (config.optional("PrunedViewMaskPathFmt")) {
      const auto &mask = prunedViewsAndMasks.second[viewId];
      IO::writeFrame(config, "PrunedViewMaskPathFmt", mask, frameIndex, viewId);
    }
  }
}

namespace {
struct Pose {
  Common::Vec3f position;
  Common::Vec3f rotation;
};

auto loadPoseFromCSV(std::istream &stream, int frameIndex) -> Pose {
  std::string line;
  getline(stream, line);

  std::regex re_header(R"(\s*X\s*,\s*Y\s*,\s*Z\s*,\s*Yaw\s*,\s*Pitch\s*,\s*Roll\s*)");
  if (!std::regex_match(line, re_header)) {
    throw std::runtime_error("Format error in the pose trace header");
  }

  int currentFrameIndex = 0;
  std::regex re_row("([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+)");
  std::regex re_empty("\\s*");
  bool trailing_empty_lines = false;

  while (getline(stream, line)) {
    std::smatch match;
    if (!trailing_empty_lines && std::regex_match(line, match, re_row)) {
      if (currentFrameIndex == frameIndex) {
        return {Common::Vec3f({stof(match[1].str()), stof(match[2].str()), stof(match[3].str())}),
                Common::Vec3f({stof(match[4].str()), stof(match[5].str()), stof(match[6].str())})};
      }
      { currentFrameIndex++; }
    } else if (std::regex_match(line, re_empty)) {
      trailing_empty_lines = true;
    } else {
      throw std::runtime_error("Format error in a pose trace row");
    }
  }

  throw std::runtime_error("Unable to load required frame index " + std::to_string(frameIndex));
}
} // namespace

auto loadViewportMetadata(const Common::Json &config, int frameIndex) -> MivBitstream::ViewParams {
  const auto cameraPath = getFullPath(config, "SourceDirectory", "SourceCameraParameters");

  std::ifstream stream{cameraPath};
  if (!stream.good()) {
    throw std::runtime_error("Failed to load camera parameters\n " + cameraPath);
  }

  auto outputviewName = config.require("OutputCameraName").as<std::string>();

  auto viewParamsList = MivBitstream::ViewParamsList::loadFromJson(
      Common::Json::loadFrom(stream).require("cameras"), {outputviewName});

  if (viewParamsList.empty()) {
    throw std::runtime_error("Unknown OutputCameraName " + outputviewName);
  }

  MivBitstream::ViewParams &result = viewParamsList.front();

  // The result may have invalid depth values
  result.hasOccupancy = true;

  if (config.optional("PoseTracePath")) {
    std::string poseTracePath = getFullPath(config, "SourceDirectory", "PoseTracePath");
    std::ifstream stream{poseTracePath};

    if (!stream.good()) {
      throw std::runtime_error("Failed to load pose trace file\n " + poseTracePath);
    }

    auto pose = loadPoseFromCSV(stream, frameIndex);

    result.ce.position(result.ce.position() + pose.position);
    result.ce.rotation(euler2quat(Common::radperdeg * pose.rotation));
  }

  return result;
}

void saveViewport(const Common::Json &config, int frameIndex,
                  const Common::TextureDepth16Frame &frame) {
  if (config.optional("OutputAttributePath")) {
    writeFrame(config, "OutputAttributePath", frame.texture, frameIndex, "T");
  }
  if (config.optional("OutputGeometryPath")) {
    writeFrame(config, "OutputGeometryPath", frame.depth, frameIndex);
  }
}
} // namespace TMIV::IO
