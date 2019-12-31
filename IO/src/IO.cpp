/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#include <TMIV/Common/Common.h>

#include <cassert>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <regex>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::IO {
auto getFullPath(const Json &config, const string &baseDirectoryField, const string &fileNameField,
                 size_t viewId, const string &viewName) -> string {
  string baseDirectory;
  string fileName =
      viewName.empty() ? format(config.require(fileNameField).asString().c_str(), viewId)
                       : format(config.require(fileNameField).asString().c_str(), viewName.c_str());

  // Detect absolute paths for /POSIX, \Windows and C:\Windows
  if ((!fileName.empty() && (fileName.front() == '/' || fileName.front() == '\\')) ||
      (fileName.size() >= 2 && fileName[1] == ':')) {
    return fileName;
  }

  if (auto subnode = config.optional(baseDirectoryField)) {
    baseDirectory = subnode.asString() + "/";
  }

  return baseDirectory + fileName;
}

namespace {
template <typename FORMAT>
auto readFrame(const string &path, int frameIndex, Vec2i resolution) -> Frame<FORMAT> {
  Frame<FORMAT> result(resolution.x(), resolution.y());
  ifstream stream{path, ifstream::binary};

  if (!stream.good()) {
    throw runtime_error("Failed to open file: " + path);
  }

  stream.seekg(streampos(frameIndex) * result.getDiskSize());
  result.read(stream);

  if (!stream.good()) {
    throw runtime_error("Failed to read from file: " + path);
  }

  return result;
}

void padZeros(ostream &stream, int bytes) {
  while (bytes-- > 0) {
    stream.put(0);
  }
}

template <typename FORMAT>
void writeFrame(const string &path, const Frame<FORMAT> &frame, int frameIndex) {
  ofstream stream(path, (frameIndex == 0 ? ios::trunc : ios::app) | ios::binary);
  if (!stream.good()) {
    throw runtime_error("Failed to open file for writing: " + path);
  }

  frame.dump(stream);
  padZeros(stream, frame.getDiskSize() - frame.getMemorySize());

  if (!stream.good()) {
    throw runtime_error("Failed to write to file: " + path);
  }
}

template <typename FORMAT>
auto loadMVDFrame(const Json &config, const SizeVector &sizes, int frameIndex, const char *what,
                  const char *directory, const char *texturePathFmt, const char *depthPathFmt,
                  const vector<string> &viewNames = {}) -> MVDFrame<FORMAT> {
  cout << "Loading " << what << " frame " << frameIndex << endl;

  MVDFrame<FORMAT> result;
  result.reserve(sizes.size());

  for (size_t i = 0; i < sizes.size(); ++i) {
    result.emplace_back(readFrame<YUV420P10>(getFullPath(config, directory, texturePathFmt, i,
                                                         viewNames.empty() ? "" : viewNames[i]),
                                             frameIndex, sizes[i]),
                        readFrame<FORMAT>(getFullPath(config, directory, depthPathFmt, i,
                                                      viewNames.empty() ? "" : viewNames[i]),
                                          frameIndex, sizes[i]));
  }

  return result;
}

template <typename FORMAT>
auto loadEntityFrame(const Json &config, const SizeVector &sizes, int frameIndex, const char *what,
                     const char *directory, const char *entityPathFmt,
                     const vector<string> &viewNames = {}) -> MEFrame<FORMAT> {
  //cout << "Loading " << what << " entity frame " << frameIndex << endl;

  MEFrame<FORMAT> result;
  result.reserve(sizes.size());

  for (size_t i = 0; i < sizes.size(); ++i) {
    result.emplace_back(readFrame<FORMAT>(
        getFullPath(config, directory, entityPathFmt, i, viewNames.empty() ? "" : viewNames[i]),
        frameIndex, sizes[i]));
  }

  return result;
}

template <typename FORMAT>
void saveMVDFrame(const Json &config, int frameIndex, const MVDFrame<FORMAT> &frame,
                  const char *what, const char *directory, const char *texturePathFmt,
                  const char *depthPathFmt) {
  cout << "Saving " << what << " frame " << frameIndex << endl;

  for (size_t i = 0; i < frame.size(); ++i) {
    writeFrame(getFullPath(config, directory, texturePathFmt, i), frame[i].first, frameIndex);
    writeFrame(getFullPath(config, directory, depthPathFmt, i), frame[i].second, frameIndex);
  }
}

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

auto loadSourceIvSequenceParams(const Json &config) -> IvSequenceParams {
  string viewPath = getFullPath(config, "SourceDirectory", "SourceCameraParameters");

  ifstream stream{viewPath};
  if (!stream.good()) {
    throw runtime_error("Failed to load source camera parameters\n" + viewPath);
  }

  const auto ivsProfileTierLevel = IvsProfileTierLevel{};

  const auto viewParamsList = ViewParamsList::loadFromJson(
      Json{stream}.require("cameras"), config.require("SourceCameraNames").asStringVector());

  const auto depthLowQualityFlag = config.require("depthLowQualityFlag").asBool();

  const auto numGroups = config.require("numGroups").asInt();
  if (numGroups < 1) {
    throw runtime_error("Require numGroups >= 1");
  }

  const auto maxEntities = config.require("maxEntities").asInt();
  if (maxEntities < 1) {
    throw runtime_error("Require maxEntities >= 1");
  }

  IvSequenceParams params = {ivsProfileTierLevel, viewParamsList, depthLowQualityFlag, unsigned(numGroups),
          unsigned(maxEntities)};

  if (auto subnode = config.optional("ViewingSpace"); subnode) {
    params.viewingSpace = ViewingSpace::loadFromJson(subnode);
  }

  return params;
}

auto loadSourceIvAccessUnitParams(const Json &config) -> Metadata::IvAccessUnitParams {
  return {AtlasParamsList{{}, config.require("OmafV1CompatibleFlag").asBool(), {}, {}, {}}};
}

namespace {
template <typename FORMAT>
auto loadSourceFrame_impl(int bits, const Json &config, const SizeVector &sizes, int frameIndex)
    -> MVD16Frame {
  auto frame = loadMVDFrame<FORMAT>(config, sizes,
                                    frameIndex + config.require("startFrame").asInt(), "source",
                                    "SourceDirectory", "SourceTexturePathFmt", "SourceDepthPathFmt",
                                    config.require("SourceCameraNames").asStringVector());
  auto frame16 = MVD16Frame{};
  frame16.reserve(frame.size());
  transform(begin(frame), end(frame), back_inserter(frame16),
            [bits](TextureDepthFrame<FORMAT> &view) {
              auto view16 = TextureDepth16Frame{
                  move(view.first), Depth16Frame{view.second.getWidth(), view.second.getHeight()}};
              transform(begin(view.second.getPlane(0)), end(view.second.getPlane(0)),
                        begin(view16.second.getPlane(0)), [bits](unsigned x) {
                          const auto x_max = maxLevel(bits);
                          assert(0 <= x && x <= x_max);
                          const auto y = (0xFFFF * x + x_max / 2) / x_max;
                          assert(0 <= y && y <= UINT16_MAX);
                          return uint16_t(y);
                        });
              return view16;
            });
  return frame16;
}

template <typename FORMAT>
auto loadSourceEntityFrame_impl(int bits, const Json &config, const SizeVector &sizes, int frameIndex)
    -> ME16Frame {
  auto frame = loadEntityFrame<FORMAT>(config, sizes,
                                    frameIndex + config.require("startFrame").asInt(), "source",
                                    "SourceDirectory", "SourceEntityPathFmt",
                                    config.require("SourceCameraNames").asStringVector());
  auto frame16 = ME16Frame{};
  frame16.reserve(frame.size());
  transform(begin(frame), end(frame), back_inserter(frame16),
            [bits](EntityFrame<FORMAT> &view) {
              auto view16 = Entity16Frame{view.getWidth(), view.getHeight()};
              transform(begin(view.getPlane(0)), end(view.getPlane(0)),
                        begin(view16.getPlane(0)), [bits](unsigned x) {
                          const auto x_max = maxLevel(bits);
                          assert(0 <= x && x <= x_max);
                          const auto y = x; // (0xFFFF * x + x_max / 2) / x_max;
                          assert(0 <= y && y <= UINT16_MAX);
                          return uint16_t(y);
                        });
              return view16;
            });
  return frame16;
}
} // namespace

auto loadSourceFrame(const Json &config, const SizeVector &sizes, int frameIndex) -> MVD16Frame {
  const auto bits = config.require("SourceDepthBitDepth").asInt();
  if (0 < bits && bits <= 8) {
    return loadSourceFrame_impl<YUV400P8>(bits, config, sizes, frameIndex);
  }
  if (8 < bits && bits <= 16) {
    return loadSourceFrame_impl<YUV400P16>(bits, config, sizes, frameIndex);
  }
  throw runtime_error("Invalid SourceDepthBitDepth");
}

auto loadSourceEntityFrame(const Json &config, const SizeVector &sizes, int frameIndex) -> ME16Frame {
  const auto bits = config.require("SourceEntityBitDepth").asInt();
  if (0 < bits && bits <= 8) {
    return loadSourceEntityFrame_impl<YUV400P8>(bits, config, sizes, frameIndex);
  }
  if (8 < bits && bits <= 16) {
    return loadSourceEntityFrame_impl<YUV400P16>(bits, config, sizes, frameIndex);
  }
  throw runtime_error("Invalid SourceEntityBitDepth");
}

void savePrunedFrame(const Json &config, int frameIndex, const MVD10Frame &frame) {
  saveMVDFrame(config, frameIndex, frame, "pruned", "OutputDirectory", "PrunedViewTexturePathFmt",
               "PrunedViewDepthPathFmt");
}

auto loadAtlas(const Json &config, const SizeVector &atlasSize, int frameIndex) -> MVD10Frame {
  return loadMVDFrame<YUV400P10>(config, atlasSize, frameIndex, "atlas", "OutputDirectory",
                                 "AtlasTexturePathFmt", "AtlasDepthPathFmt");
}

void saveAtlas(const Json &config, int frameIndex, const MVD10Frame &frame) {
  saveMVDFrame(config, frameIndex, frame, "atlas", "OutputDirectory", "AtlasTexturePathFmt",
               "AtlasDepthPathFmt");
}

auto loadPatchIdMaps(const Json &config, const SizeVector &atlasSize, int frameIndex)
    -> PatchIdMapList {
  cout << "Loading patchIdMap frame " << frameIndex << '\n';

  PatchIdMapList result;

  for (size_t id = 0; id < atlasSize.size(); ++id) {
    string texturePath = getFullPath(config, "OutputDirectory", "AtlasPatchOccupancyMapFmt", id);
    auto textureFrame = readFrame<YUV400P16>(texturePath, frameIndex, atlasSize[id]);

    result.push_back(move(textureFrame));
  }

  return result;
}

void savePatchIdMaps(const Json &config, int frameIndex, const PatchIdMapList &maps) {
  cout << "Saving patchIdMap frame " << frameIndex << '\n';

  for (size_t id = 0; id < maps.size(); ++id) {
    string texturePath = getFullPath(config, "OutputDirectory", "AtlasPatchOccupancyMapFmt", id);
    writeFrame(texturePath, maps[id], frameIndex);
  }
}

auto loadViewportMetadata(const Json &config, int frameIndex) -> ViewParams {

  string cameraPath = getFullPath(config, "SourceDirectory", "SourceCameraParameters");

  ifstream stream{cameraPath};
  if (!stream.good()) {
    throw runtime_error("Failed to load camera parameters\n " + cameraPath);
  }

  auto outputviewName = config.require("OutputCameraName").asString();

  auto viewParamsVector =
      ViewParamsList::loadFromJson(Json{stream}.require("cameras"), {outputviewName});

  if (viewParamsVector.empty()) {
    throw runtime_error("Unknown OutputCameraName " + outputviewName);
  }

  ViewParams &result = viewParamsVector.front();

  // Override hasInvalidDepth parameter to be true because view synthesis may result in invalid
  // depth values
  result.depthOccMapThreshold = 1;

  if (auto nodeOutputCameraPoseTrace = config.optional("PoseTracePath")) {
    string poseTracePath = getFullPath(config, "SourceDirectory", "PoseTracePath");
    ifstream stream{poseTracePath};

    if (!stream.good()) {
      throw runtime_error("Failed to load pose trace file\n " + poseTracePath);
    }

    auto pose = loadPoseFromCSV(stream, frameIndex);

    result.position += pose.position;
    result.rotation = pose.rotation;
  }

  return result;
}

void saveViewport(const Json &config, int frameIndex, const TextureDepth16Frame &frame) {
  cout << "Saving viewport frame " << frameIndex << '\n';

  string texturePath = getFullPath(config, "OutputDirectory", "OutputTexturePath", 0,
                                   config.require("OutputCameraName").asString());
  writeFrame(texturePath, frame.first, frameIndex);

  if (config.optional("OutputDepthPath")) {
    string depthPath = getFullPath(config, "OutputDirectory", "OutputDepthPath", 0,
                                   config.require("OutputCameraName").asString());
    writeFrame(depthPath, frame.second, frameIndex);
  }
}

auto getExtendedIndex(const Json &config, int frameIndex) -> int {
  int numberOfFrames = config.require("numberOfFrames").asInt();
  int frameGroupId = frameIndex / numberOfFrames;
  int frameRelativeId = frameIndex % numberOfFrames;
  return (frameGroupId % 2) != 0 ? (numberOfFrames - (frameRelativeId + 1)) : frameRelativeId;
}
} // namespace TMIV::IO
