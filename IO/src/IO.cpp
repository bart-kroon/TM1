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

#include <cassert>
#include <functional>
#include <iomanip>
#include <iostream>
#include <regex>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::IO {
namespace {
// The TMIV encoder always loads texture (and may use it internally) but attribute video data (AVD)
// is an optional output in MIV WD4.
auto haveTexture(const Json &config) { return !config.optional("noTexture"); }
} // namespace

auto loadSourceIvSequenceParams(const Json &config) -> IvSequenceParams {
  auto x = IvSequenceParams{haveTexture(config)};

  string viewPath = getFullPath(config, "SourceDirectory", "SourceCameraParameters");

  ifstream stream{viewPath};
  if (!stream.good()) {
    throw runtime_error("Failed to load source camera parameters\n" + viewPath);
  }
  const auto sequenceConfig = Json{stream};

  x.viewParamsList = ViewParamsList::loadFromJson(
      sequenceConfig.require("cameras"), config.require("SourceCameraNames").asStringVector());

  if (config.isPresent("depthLowQualityFlag")) {
    auto node = config.optional("depthLowQualityFlag");
    x.msp().msp_depth_low_quality_flag(node.asBool());
  }

  const auto numGroups = unsigned(config.require("numGroups").asInt());
  if (numGroups < 1) {
    throw runtime_error("Require numGroups >= 1");
  }
  x.msp().msp_num_groups_minus1(numGroups - 1U);

  const auto maxEntities = unsigned(config.require("maxEntities").asInt());
  if (maxEntities < 1) {
    throw runtime_error("Require maxEntities >= 1");
  }
  x.msp().msp_max_entities_minus1(maxEntities - 1U);

  if (auto subnode = config.optional("ViewingSpace"); subnode) {
    x.viewingSpace = ViewingSpace::loadFromJson(subnode);
  }

  x.frameRate = sequenceConfig.require("Fps").asDouble();

  return x;
}

auto loadSourceIvAccessUnitParams(const Json &config) -> IvAccessUnitParams {
  auto x = IvAccessUnitParams{};

  x.atlas.emplace_back();
  x.atlas.front()
      .asps.asps_use_eight_orientations_flag(true)
      .asps_extension_present_flag(true)
      .asps_miv_extension_present_flag(true)
      .miv_atlas_sequence_params()
      .masp_omaf_v1_compatible_flag(config.require("OmafV1CompatibleFlag").asBool());

  return x;
}

namespace {
auto loadSourceTexture(const Json &config, const Vec2i &size, const string &viewName,
                       int frameIndex) {
  return readFrame<YUV420P10>(config, "SourceDirectory", "SourceTexturePathFmt", frameIndex, size,
                              viewName);
}

template <typename FORMAT>
auto loadSourceDepth_(int bits, const Json &config, const Vec2i &size, const string &viewName,
                      int frameIndex) {
  auto depth16 = Depth16Frame{size.x(), size.y()};

  const auto depth = readFrame<FORMAT>(config, "SourceDirectory", "SourceDepthPathFmt", frameIndex,
                                       size, viewName);

  transform(begin(depth.getPlane(0)), end(depth.getPlane(0)), begin(depth16.getPlane(0)),
            [bits](unsigned x) {
              const auto x_max = maxLevel(bits);
              assert(0 <= x && x <= x_max);
              const auto y = (0xFFFF * x + x_max / 2) / x_max;
              assert(0 <= y && y <= UINT16_MAX);
              return uint16_t(y);
            });

  return depth16;
}

auto loadSourceDepth(const Json &config, const Vec2i &size, const string &viewName,
                     int frameIndex) {
  const auto bits = config.require("SourceDepthBitDepth").asInt();

  if (0 < bits && bits <= 8) {
    return loadSourceDepth_<YUV400P8>(bits, config, size, viewName, frameIndex);
  }
  if (8 < bits && bits <= 16) {
    return loadSourceDepth_<YUV400P16>(bits, config, size, viewName, frameIndex);
  }
  throw runtime_error("Invalid SourceDepthBitDepth");
}

template <typename FORMAT>
auto loadSourceEntities_(const Json &config, const Vec2i size, const string &viewName,
                         int frameIndex) {
  auto entities16 = EntityMap{size.x(), size.y()};

  const auto entities = readFrame<FORMAT>(config, "SourceDirectory", "SourceEntityPathFmt",
                                          frameIndex, size, viewName);

  copy(entities.getPlane(0).begin(), entities.getPlane(0).end(), entities16.getPlane(0).begin());

  return entities16;
}

auto loadSourceEntities(const Json &config, const Vec2i size, const string &viewName,
                        int frameIndex) {
  if (auto node = config.optional("SourceEntityBitDepth"); node) {
    const auto bits = node.asInt();
    if (0 < bits && bits <= 8) {
      return loadSourceEntities_<YUV400P8>(config, size, viewName, frameIndex);
    }
    if (8 < bits && bits <= 16) {
      return loadSourceEntities_<YUV400P16>(config, size, viewName, frameIndex);
    }
    throw runtime_error("Invalid SourceEntityBitDepth");
  }
  return EntityMap{};
}
} // namespace

auto loadSourceFrame(const Json &config, const SizeVector &sizes, int frameIndex) -> MVD16Frame {
  auto frame = MVD16Frame(sizes.size());

  frameIndex += config.require("startFrame").asInt();

  const auto viewNames = config.require("SourceCameraNames").asStringVector();
  assert(viewNames.size() == sizes.size());

  for (size_t viewId = 0; viewId < frame.size(); ++viewId) {
    auto &view = frame[viewId];
    view.texture = loadSourceTexture(config, sizes[viewId], viewNames[viewId], frameIndex);
    view.depth = loadSourceDepth(config, sizes[viewId], viewNames[viewId], frameIndex);
    view.entities = loadSourceEntities(config, sizes[viewId], viewNames[viewId], frameIndex);
  }

  return frame;
}

void saveAtlas(const Json &config, int frameIndex, const MVD10Frame &frame) {
  for (size_t atlasId = 0; atlasId < frame.size(); ++atlasId) {
    if (haveTexture(config)) {
      writeFrame(config, "AttributeVideoDataPathFmt", frame[atlasId].texture, frameIndex,
                 int(atlasId));
    }
    writeFrame(config, "GeometryVideoDataPathFmt", frame[atlasId].depth, frameIndex, int(atlasId));
  }
}

void saveBlockToPatchMaps(const Json &config, int frameIndex, const AccessUnit &frame) {
  for (size_t atlasId = 0; atlasId < frame.atlas.size(); ++atlasId) {
    writeFrame(config, "AtlasPatchOccupancyMapFmt", frame.atlas[atlasId].blockToPatchMap,
               frameIndex, int(atlasId));
  }
}

void savePrunedFrame(const Json &config, int frameIndex,
                     const pair<vector<Texture444Depth10Frame>, MaskList> &prunedViewsAndMasks) {
  for (size_t viewId = 0; viewId < prunedViewsAndMasks.first.size(); ++viewId) {
    const auto &view = prunedViewsAndMasks.first[viewId];
    if (config.optional("PrunedViewTexturePathFmt")) {
      IO::writeFrame(config, "PrunedViewTexturePathFmt", view.first, frameIndex, viewId);
    }
    if (config.optional("PrunedViewDepthPathFmt")) {
      IO::writeFrame(config, "PrunedViewDepthPathFmt", view.second, frameIndex, viewId);
    }
    if (config.optional("PrunedViewMaskPathFmt")) {
      const auto &mask = prunedViewsAndMasks.second[viewId];
      IO::writeFrame(config, "PrunedViewMaskPathFmt", mask, frameIndex, viewId);
    }
  }
}

namespace {
struct Pose {
  Vec3f position;
  Vec3f rotation;
};

auto loadPoseFromCSV(istream &stream, int frameIndex) -> Pose {
  string line;
  getline(stream, line);

  regex re_header(R"(\s*X\s*,\s*Y\s*,\s*Z\s*,\s*Yaw\s*,\s*Pitch\s*,\s*Roll\s*)");
  if (!regex_match(line, re_header)) {
    throw runtime_error("Format error in the pose trace header");
  }

  int currentFrameIndex = 0;
  regex re_row("([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+)");
  regex re_empty("\\s*");
  bool trailing_empty_lines = false;

  while (getline(stream, line)) {
    smatch match;
    if (!trailing_empty_lines && regex_match(line, match, re_row)) {

      if (currentFrameIndex == frameIndex) {
        return {Vec3f({stof(match[1].str()), stof(match[2].str()), stof(match[3].str())}),
                Vec3f({stof(match[4].str()), stof(match[5].str()), stof(match[6].str())})};
      }
      { currentFrameIndex++; }
    } else if (regex_match(line, re_empty)) {
      trailing_empty_lines = true;
    } else {
      throw runtime_error("Format error in a pose trace row");
    }
  }

  throw runtime_error("Unable to load required frame index " + to_string(frameIndex));
}
} // namespace

auto loadViewportMetadata(const Json &config, int frameIndex) -> ViewParams {
  const auto cameraPath = getFullPath(config, "SourceDirectory", "SourceCameraParameters");

  ifstream stream{cameraPath};
  if (!stream.good()) {
    throw runtime_error("Failed to load camera parameters\n " + cameraPath);
  }

  auto outputviewName = config.require("OutputCameraName").asString();

  auto viewParamsList =
      ViewParamsList::loadFromJson(Json{stream}.require("cameras"), {outputviewName});

  if (viewParamsList.empty()) {
    throw runtime_error("Unknown OutputCameraName " + outputviewName);
  }

  ViewParams &result = viewParamsList.front();

  // The result may have invalid depth values
  result.hasOccupancy = true;

  if (auto nodeOutputCameraPoseTrace = config.optional("PoseTracePath")) {
    string poseTracePath = getFullPath(config, "SourceDirectory", "PoseTracePath");
    ifstream stream{poseTracePath};

    if (!stream.good()) {
      throw runtime_error("Failed to load pose trace file\n " + poseTracePath);
    }

    auto pose = loadPoseFromCSV(stream, frameIndex);

    result.ce.position(result.ce.position() + pose.position);
    result.ce.rotation(euler2quat(radperdeg * pose.rotation));
  }

  return result;
}

void saveViewport(const Json &config, int frameIndex, const TextureDepth16Frame &frame) {
  if (config.optional("OutputTexturePath")) {
    writeFrame(config, "OutputTexturePath", frame.texture, frameIndex);
  }
  if (config.optional("OutputDepthPath")) {
    writeFrame(config, "OutputDepthPath", frame.depth, frameIndex);
  }
}
} // namespace TMIV::IO
