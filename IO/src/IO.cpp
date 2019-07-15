/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <regex>

#include <TMIV/Common/Common.h>
#include <TMIV/Image/Image.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Image;

namespace TMIV::IO {
namespace {
string getFullPath(const Json &config, const string &baseDirectoryField,
                   const string &fileNameField, size_t cameraId = 0,
                   const std::string &cameraName = "") {
  string baseDirectory,
      fileName = cameraName.empty()
                     ? format(config.require(fileNameField).asString().c_str(),
                              cameraId)
                     : format(config.require(fileNameField).asString().c_str(),
                              cameraName.c_str());

  if (!fileName.empty() && fileName.front() == '/') {
    return fileName;
  }

  if (auto subnode = config.optional(baseDirectoryField)) {
    baseDirectory = subnode.asString() + "/";
  }

  return baseDirectory + fileName;
}

template <typename FORMAT>
Frame<FORMAT> readFrame(const string &path, int frameIndex, Vec2i resolution) {
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
void writeFrame(const string &path, const Frame<FORMAT> &frame,
                int frameIndex) {
  ofstream stream(path,
                  (frameIndex == 0 ? ios::trunc : ios::app) | ios::binary);
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
MVDFrame<FORMAT>
loadMVDFrame(const Json &config, const vector<Vec2i> &sizes, int frameIndex,
             const char *what, const char *directory,
             const char *texturePathFmt, const char *depthPathFmt,
             const std::vector<std::string> &cameraNames = {}) {
  cout << "Loading " << what << " frame " << frameIndex << endl;

  MVDFrame<FORMAT> result;
  result.reserve(sizes.size());

  for (size_t i = 0u; i < sizes.size(); ++i) {

    result.emplace_back(
        readFrame<YUV420P10>(
            getFullPath(config, directory, texturePathFmt, i,
                        cameraNames.empty() ? "" : cameraNames[i]),
            frameIndex, sizes[i]),
        readFrame<FORMAT>(
            getFullPath(config, directory, depthPathFmt, i,
                        cameraNames.empty() ? "" : cameraNames[i]),
            frameIndex, sizes[i]));
  }

  return result;
}

template <typename FORMAT>
void saveMVDFrame(const Json &config, int frameIndex,
                  const MVDFrame<FORMAT> &frame, const char *what,
                  const char *directory, const char *texturePathFmt,
                  const char *depthPathFmt) {
  cout << "Saving " << what << " frame " << frameIndex << endl;

  for (size_t i = 0u; i < frame.size(); ++i) {
    writeFrame(getFullPath(config, directory, texturePathFmt, i),
               frame[i].first, frameIndex);
    writeFrame(getFullPath(config, directory, depthPathFmt, i), frame[i].second,
               frameIndex);
  }
}

CameraParameters readCameraFromFile(istream &is) {
  CameraParameters camera;
  is.read(reinterpret_cast<char *>(&camera), sizeof(camera));
  return camera;
}

CameraParametersList readCameraListFromFile(istream &is) {
  uint16_t nbCamera = 0;
  CameraParametersList list;

  is.read(reinterpret_cast<char *>(&nbCamera), sizeof(uint16_t));

  for (auto i = 0; i < nbCamera; i++) {
    list.push_back(readCameraFromFile(is));
  }

  return list;
}

void skipCameraListFromFile(istream &is) {
  uint16_t nbCamera = 0;
  is.read(reinterpret_cast<char *>(&nbCamera), sizeof(uint16_t));

  is.seekg(nbCamera * sizeof(CameraParameters), ios::cur);
}

void writeCameraToFile(ofstream &os, const CameraParameters &camera) {
  os.write(reinterpret_cast<const char *>(&camera), sizeof(camera));
}

void writeCameraListToFile(ofstream &os, const CameraParametersList &list) {
  auto nbCamera = uint16_t(list.size());

  os.write(reinterpret_cast<char *>(&nbCamera), sizeof(uint16_t));

  for (const auto &camera : list) {
    writeCameraToFile(os, camera);
  }
}

vector<Vec2i> readAtlasSizeFromFile(ifstream &is) {
  uint8_t nbAtlas = 0;
  vector<Vec2i> result;

  is.read(reinterpret_cast<char *>(&nbAtlas), sizeof(uint8_t));

  result.resize(nbAtlas);
  is.read(reinterpret_cast<char *>(result.data()), nbAtlas * sizeof(Vec2i));

  return result;
}

void skipAtlasSizeFromFile(ifstream &is) {
  uint8_t nbAtlas = 0;

  is.read(reinterpret_cast<char *>(&nbAtlas), sizeof(uint8_t));

  is.seekg(nbAtlas * sizeof(Vec2i), ios::cur);
}

void writeAtlasSizeToFile(ofstream &os, const vector<Vec2i> &atlasSize) {
  auto nbAtlas = uint8_t(atlasSize.size());

  os.write(reinterpret_cast<const char *>(&nbAtlas), sizeof(uint8_t));

  os.write(reinterpret_cast<const char *>(atlasSize.data()),
           nbAtlas * sizeof(Vec2i));
}

bool readFlagFromFile(ifstream &is) {
  char flag{};
  is.read(&flag, 1);
  return !!flag;
}

void skipFlagFromFile(ifstream &is) { is.seekg(1, ios::cur); }

void writeFlagToFile(ofstream &os, bool flag) {
  auto flag_c = flag ? '\x1' : '\x0';
  os.write(&flag_c, 1);
}

AtlasParameters readPatchFromFile(ifstream &is) {
  AtlasParameters patch;
  is.read(reinterpret_cast<char *>(&patch), sizeof(patch));
  return patch;
}

AtlasParametersList readPatchListFromFile(ifstream &is) {
  uint16_t nbPatch = 0;
  AtlasParametersList list;

  is.read(reinterpret_cast<char *>(&nbPatch), sizeof(uint16_t));

  for (auto i = 0; i < nbPatch; i++) {
    list.push_back(readPatchFromFile(is));
  }

  return list;
}

void skipPatchListFromFile(istream &is) {
  uint16_t nbPatch = 0;

  is.read(reinterpret_cast<char *>(&nbPatch), sizeof(uint16_t));

  is.seekg(nbPatch * sizeof(AtlasParameters), ios::cur);
}

void writePatchToFile(ofstream &os, const AtlasParameters &patch) {
  os.write(reinterpret_cast<const char *>(&patch), sizeof(patch));
}

void writePatchListToFile(ofstream &os, const AtlasParametersList &list) {
  auto nbPatch = uint16_t(list.size());

  os.write(reinterpret_cast<char *>(&nbPatch), sizeof(uint16_t));

  for (const auto &patch : list) {
    writePatchToFile(os, patch);
  }
}

template <typename T>
T readMetadataFromFile(const string &path, int frameIndex,
                       const function<void(ifstream &)> &skipFunction,
                       function<T(ifstream &)> readFunction) {
  ifstream stream{path, ios::binary};

  if (!stream.good()) {
    throw runtime_error("Failed to open file: " + path);
  }

  // Seeking
  stream.seekg(streamoff(0), ifstream::beg);

  while (true) {
    uint32_t frameId = 0;
    stream.read(reinterpret_cast<char *>(&frameId), sizeof(uint32_t));

    if (!stream.good()) {
      throw runtime_error("Failed to read frame #" + to_string(frameIndex) +
                          " from file: " + path);
    }

    if (frameId != uint32_t(frameIndex)) {
      skipFunction(stream);
    } else {
      break;
    }
  }

  // Reading
  return readFunction(stream);
}

template <typename T>
void writeMetadataToFile(const string &path, int frameIndex, const T &metadata,
                         function<void(ofstream &, const T &)> writeFunction) {
  ofstream stream{path,
                  (frameIndex == 0 ? ios::trunc : ios::app) | ios::binary};

  if (!stream.good()) {
    throw runtime_error("Failed to open file: " + path);
  }

  // Frame index
  uint32_t frameId = frameIndex;
  stream.write(reinterpret_cast<const char *>(&frameId), sizeof(uint32_t));

  if (!stream.good()) {
    throw runtime_error("Failed to write frame #" + to_string(frameIndex) +
                        " to file: " + path);
  }

  // Metadata
  writeFunction(stream, metadata);
}

struct Pose {
  Vec3f position;
  Vec3f rotation;
};

Pose loadPoseFromCSV(std::istream &stream, int frameIndex) {
  std::string line;
  std::getline(stream, line);

  std::regex re_header(
      R"(\s*X\s*,\s*Y\s*,\s*Z\s*,\s*Yaw\s*,\s*Pitch\s*,\s*Roll\s*)");
  if (!std::regex_match(line, re_header)) {
    throw std::runtime_error("Format error in the pose trace header");
  }

  int currentFrameIndex = 0;
  std::regex re_row("([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+)");
  std::regex re_empty("\\s*");
  bool trailing_empty_lines = false;

  while (std::getline(stream, line)) {
    std::smatch match;
    if (!trailing_empty_lines && std::regex_match(line, match, re_row)) {

      if (currentFrameIndex == frameIndex) {
        return {Vec3f({std::stof(match[1].str()), std::stof(match[2].str()),
                       std::stof(match[3].str())}),
                Vec3f({std::stof(match[4].str()), std::stof(match[5].str()),
                       std::stof(match[6].str())})};
      }
      { currentFrameIndex++; }
    } else if (std::regex_match(line, re_empty)) {
      trailing_empty_lines = true;
    } else {
      throw std::runtime_error("Format error in a pose trace row");
    }
  }

  throw std::runtime_error("Unable to load required frame index " +
                           to_string(frameIndex));
}

} // namespace

auto sizesOf(const CameraParametersList &cameras) -> vector<Vec2i> {
  vector<Vec2i> sizes;
  sizes.reserve(cameras.size());
  transform(begin(cameras), end(cameras), back_inserter(sizes),
            [](const CameraParameters &camera) { return camera.size; });
  return sizes;
}

CameraParametersList loadSourceMetadata(const Json &config) {
  cout << "Loading source metadata\n";

  string cameraPath =
      getFullPath(config, "SourceDirectory", "SourceCameraParameters");
  ifstream stream{cameraPath};

  if (!stream.good()) {
    throw runtime_error("Failed to load source camera parameters\n" +
                        cameraPath);
  }

  auto cameras =
      loadCamerasFromJson(Json{stream}.require("cameras"),
                          config.require("SourceCameraNames").asStringVector());

  for (size_t i = 0u; i < cameras.size(); ++i) {
    cout << "Camera " << setw(2) << i << ": " << cameras[i] << '\n';
  }

  return cameras;
}

MVD16Frame loadSourceFrame(const Json &config, const vector<Vec2i> &sizes,
                           int frameIndex) {
  auto sourceCameraNames = config.require("SourceCameraNames").asStringVector();

  frameIndex += config.require("startFrame").asInt();
  return loadMVDFrame<YUV400P16>(config, sizes, frameIndex, "source",
                                 "SourceDirectory", "SourceTexturePathFmt",
                                 "SourceDepthPathFmt", sourceCameraNames);
}

BasicAdditional<CameraParametersList> loadOptimizedMetadata(const Json &config,
                                                            int frameIndex) {
  cout << "Loading optimized metadata\n";

  BasicAdditional<CameraParametersList> result;
  string basicMetadataPath =
      getFullPath(config, "OutputDirectory", "BasicMetadataPath");
  string additionalMetadataPath =
      getFullPath(config, "OutputDirectory", "AdditionalMetadataPath");

  auto skipFunction = [](ifstream &is) { skipCameraListFromFile(is); };

  auto readFunction = [](ifstream &is) -> CameraParametersList {
    return readCameraListFromFile(is);
  };

  // Reading
  return BasicAdditional<CameraParametersList>{
      readMetadataFromFile<CameraParametersList>(basicMetadataPath, frameIndex,
                                                 skipFunction, readFunction),
      readMetadataFromFile<CameraParametersList>(
          additionalMetadataPath, frameIndex, skipFunction, readFunction)};
}

void saveOptimizedMetadata(
    const Json &config, int frameIndex,
    const BasicAdditional<CameraParametersList> &metadata) {
  cout << "Saving metadata of optimized frame " << frameIndex << '\n';

  string basicMetadataPath =
      getFullPath(config, "OutputDirectory", "BasicMetadataPath");
  string additionalMetadataPath =
      getFullPath(config, "OutputDirectory", "AdditionalMetadataPath");

  auto writeFunction = [](ofstream &os, const CameraParametersList &metadata) {
    writeCameraListToFile(os, metadata);
  };

  writeMetadataToFile<CameraParametersList>(basicMetadataPath, frameIndex,
                                            metadata.basic, writeFunction);
  writeMetadataToFile<CameraParametersList>(additionalMetadataPath, frameIndex,
                                            metadata.additional, writeFunction);
}

BasicAdditional<MVD16Frame>
loadOptimizedFrame(const Json &config,
                   const BasicAdditional<vector<Vec2i>> &sizes,
                   int frameIndex) {
  return {loadMVDFrame<YUV400P16>(config, sizes.basic, frameIndex,
                                  "basic views of", "OutputDirectory",
                                  "BasicTexturePathFmt", "BasicDepthPathFmt"),
          loadMVDFrame<YUV400P16>(config, sizes.additional, frameIndex,
                                  "additional views of", "OutputDirectory",
                                  "AdditionalTexturePathFmt",
                                  "AdditionalDepthPathFmt")};
}

void saveOptimizedFrame(const Json &config, int frameIndex,
                        const BasicAdditional<MVD16Frame> &frame) {
  saveMVDFrame(config, frameIndex, frame.basic, "basic views of",
               "OutputDirectory", "BasicTexturePathFmt", "BasicDepthPathFmt");
  saveMVDFrame(config, frameIndex, frame.additional, "additional views of",
               "OutputDirectory", "AdditionalTexturePathFmt",
               "AdditionalDepthPathFmt");
}

void savePrunedFrame(const Json &config, int frameIndex,
                     const MVD16Frame &frame) {
  saveMVDFrame(config, frameIndex, frame, "pruned", "OutputDirectory",
               "PrunedViewTexturePathFmt", "PrunedViewDepthPathFmt");
}

MivMetadata loadMivMetadata(const Json &config, int frameIndex) {
  cout << "Loading MIV metadata of frame " << frameIndex << '\n';

  MivMetadata result;
  string metadataPath =
      getFullPath(config, "OutputDirectory", "AtlasMetadataPath");

  auto skipFunction = [](ifstream &is) {
    skipAtlasSizeFromFile(is);
    skipFlagFromFile(is); // OMAF v1 compatible flag
    skipPatchListFromFile(is);
    skipCameraListFromFile(is);
  };

  auto readFunction = [](ifstream &is) -> MivMetadata {
    return MivMetadata{readAtlasSizeFromFile(is), readFlagFromFile(is),
                       readPatchListFromFile(is), readCameraListFromFile(is)};
  };

  // Reading
  return readMetadataFromFile<MivMetadata>(metadataPath, frameIndex,
                                           skipFunction, readFunction);
}

bool MivMetadata::operator==(const MivMetadata &other) const {
  return atlasSize == other.atlasSize &&
         omafV1CompatibleFlag == other.omafV1CompatibleFlag &&
         patches == other.patches && cameras == other.cameras;
}

void saveMivMetadata(const Json &config, int frameIndex,
                     const MivMetadata &metadata) {
  cout << "Saving MIV metadata of frame " << frameIndex << '\n';

  string metadataPath =
      getFullPath(config, "OutputDirectory", "AtlasMetadataPath");

  auto writeFunction = [](ofstream &os, const MivMetadata &metadata) {
    writeAtlasSizeToFile(os, metadata.atlasSize);
    writeFlagToFile(os, metadata.omafV1CompatibleFlag);
    writePatchListToFile(os, metadata.patches);
    writeCameraListToFile(os, metadata.cameras);
  };

  writeMetadataToFile<MivMetadata>(metadataPath, frameIndex, metadata,
                                   writeFunction);
}

void savePatchList(const Json &config, const string &name,
                   Metadata::AtlasParametersList patches) {

  string baseDirectory = config.require("OutputDirectory").asString();
  string path = baseDirectory + name;

  ofstream os(path);
  if (!os.good()) {
    throw runtime_error("Failed to open file for writing: " + path);
  }

  int idx = 0;
  for (const auto &p : patches) {
    os << idx++ << ": " << PatchParametersString(p) << endl;
  }

  os.close();
}

MVD10Frame loadAtlas(const Json &config, const vector<Vec2i> &atlasSize,
                     int frameIndex) {
  return loadMVDFrame<YUV400P10>(config, atlasSize, frameIndex, "atlas",
                                 "OutputDirectory", "AtlasTexturePathFmt",
                                 "AtlasDepthPathFmt");
}

void saveAtlas(const Json &config, int frameIndex, MVD16Frame frame) {
  // Convert from 16 to 10-bit depth
  MVD10Frame frame10;
  frame10.reserve(frame.size());
  transform(begin(frame), end(frame), back_inserter(frame10),
            [](TextureDepth16Frame &view) {
              return pair{move(view.first), requantize10(view.second)};
            });

  saveAtlas(config, frameIndex, frame10);
}

void saveAtlas(const Json &config, int frameIndex, const MVD10Frame &frame) {
  saveMVDFrame(config, frameIndex, frame, "atlas", "OutputDirectory",
               "AtlasTexturePathFmt", "AtlasDepthPathFmt");
}

PatchIdMapList loadPatchIdMaps(const Json &config,
                               const vector<Vec2i> &atlasSize, int frameIndex) {
  cout << "Loading patchIdMap frame " << frameIndex << '\n';

  PatchIdMapList result;

  for (auto id = 0u; id < atlasSize.size(); id++) {
    string texturePath =
        getFullPath(config, "OutputDirectory", "AtlasPatchOccupancyMapFmt", id);
    auto textureFrame =
        readFrame<YUV400P16>(texturePath, frameIndex, atlasSize[id]);

    result.push_back(move(textureFrame));
  }

  return result;
}

void savePatchIdMaps(const Json &config, int frameIndex,
                     const PatchIdMapList &maps) {
  cout << "Saving patchIdMap frame " << frameIndex << '\n';

  for (auto id = 0u; id < maps.size(); id++) {
    string texturePath =
        getFullPath(config, "OutputDirectory", "AtlasPatchOccupancyMapFmt", id);
    writeFrame(texturePath, maps[id], frameIndex);
  }
}

CameraParameters loadViewportMetadata(const Json &config, int frameIndex) {

  CameraParameters result;

  string cameraPath =
      getFullPath(config, "SourceDirectory", "SourceCameraParameters");

  ifstream stream{cameraPath};
  if (!stream.good()) {
    throw runtime_error("Failed to load camera parameters\n " + cameraPath);
  }

  auto outputCameraName = config.require("OutputCameraName").asString();

  auto cameras =
      loadCamerasFromJson(Json{stream}.require("cameras"), {outputCameraName});

  if (cameras.empty()) {
    throw runtime_error("Unknown OutputCameraName " + outputCameraName);
  }

  result = cameras[0];

  if (auto nodeOutputCameraPoseTrace = config.optional("PoseTracePath")) {
    string poseTracePath =
        getFullPath(config, "SourceDirectory", "PoseTracePath");
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

void saveViewport(const Json &config, int frameIndex,
                  const TextureDepth10Frame &frame) {
  cout << "Saving viewport frame " << frameIndex << '\n';

  string texturePath =
      getFullPath(config, "OutputDirectory", "OutputTexturePath", 0,
                  config.require("OutputCameraName").asString());
  writeFrame(texturePath, frame.first, frameIndex);

  if (config.optional("OutputDepthPath")) {
    string depthPath =
        getFullPath(config, "OutputDirectory", "OutputDepthPath", 0,
                    config.require("OutputCameraName").asString());
    writeFrame(depthPath, frame.second, frameIndex);
  }
}

pair<int, int> getExtendedIndex(const Json &config, int frameIndex) {
  int numberOfFrames = config.require("numberOfFrames").asInt();
  int intraPeriod = config.require("intraPeriod").asInt();

  int frameGroupId = frameIndex / numberOfFrames;
  int frameRelativeId = frameIndex % numberOfFrames;

  int frameIndexExtended = (frameGroupId % 2) != 0
                               ? (numberOfFrames - (frameRelativeId + 1))
                               : frameRelativeId;
  int metadataIndexExtended = frameIndexExtended / intraPeriod;

  return {metadataIndexExtended * intraPeriod, frameIndexExtended};
}
} // namespace TMIV::IO
