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
#include <iostream>

#include <TMIV/Common/Common.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::IO {

namespace {

std::string getFullPath(const Json &config,
                        const std::string &baseDirectoryField,
                        const std::string &fileNameField, size_t cameraId = 0) {
  std::string baseDirectory,
      fileName =
          format(config.require(fileNameField).asString().c_str(), cameraId);

  if (auto subnode = config.optional(baseDirectoryField))
    baseDirectory = subnode.asString() + "/";

  return baseDirectory + fileName;
}

template <typename FORMAT>
Frame<FORMAT> readFrame(const std::string &path, int frameIndex,
                        Vec2i resolution) {
  Frame<FORMAT> result(resolution.x(), resolution.y());
  std::ifstream stream{path, std::ifstream::binary};

  if (!stream.good())
    throw runtime_error("Failed to open file: " + path);

  stream.seekg(streampos(frameIndex) * result.getMemorySize());
  result.read(stream);

  if (!stream.good())
    throw runtime_error("Failed to read from file: " + path);

  return result;
}

template <typename FORMAT>
void writeFrame(const std::string &path, const Frame<FORMAT> &frame,
                bool trunc) {
  std::ofstream stream(path,
                       (trunc ? std::ofstream::trunc : std::ofstream::app) |
                           std::ofstream::binary);

  if (!stream.good())
    throw runtime_error("Failed to open file for writing: " + path);

  frame.dump(stream);

  if (!stream.good())
    throw runtime_error("Failed to write to file: " + path);
}

CameraParameters readCameraFromFile(std::istream &is) {
  CameraParameters camera;

  is.read((char *)&(camera.id), sizeof(uint16_t));
  is.read((char *)&(camera.size), sizeof(Vec2i));
  is.read((char *)&(camera.position), sizeof(Vec3f));
  is.read((char *)&(camera.rotation), sizeof(Vec3f));
  is.read((char *)&(camera.type), sizeof(ProjectionType));
  is.read((char *)&(camera.erpPhiRange), sizeof(Vec2f));
  is.read((char *)&(camera.erpThetaRange), sizeof(Vec2f));
  is.read((char *)&(camera.cubicMapType), sizeof(CubicMapType));
  is.read((char *)&(camera.perspectiveFocal), sizeof(Vec2f));
  is.read((char *)&(camera.perspectiveCenter), sizeof(Vec2f));
  is.read((char *)&(camera.depthRange), sizeof(Vec2f));

  return camera;
}

CameraParameterList readCameraListFromFile(std::istream &is) {
  uint16_t nbCamera = 0;
  CameraParameterList list;

  is.read((char *)&nbCamera, sizeof(uint16_t));

  for (auto i = 0; i < nbCamera; i++)
    list.push_back(readCameraFromFile(is));

  return list;
}

void skipCameraListFromFile(std::istream &is) {
  static const size_t cameraSizeInFile =
      sizeof(uint16_t) + sizeof(Vec2i) + sizeof(Vec3f) + sizeof(Vec3f) +
      sizeof(ProjectionType) + sizeof(Vec2f) + sizeof(Vec2f) +
      sizeof(CubicMapType) + sizeof(Vec2f) + sizeof(Vec2f) + sizeof(Vec2f);

  uint16_t nbCamera = 0;

  is.read((char *)&nbCamera, sizeof(uint16_t));

  is.seekg(nbCamera * cameraSizeInFile, std::ifstream::cur);
}

void writeCameraToFile(std::ofstream &os, const CameraParameters &camera) {
  os.write((const char *)&(camera.id), sizeof(uint16_t));
  os.write((const char *)&(camera.size), sizeof(Vec2i));
  os.write((const char *)&(camera.position), sizeof(Vec3f));
  os.write((const char *)&(camera.rotation), sizeof(Vec3f));
  os.write((const char *)&(camera.type), sizeof(ProjectionType));
  os.write((const char *)&(camera.erpPhiRange), sizeof(Vec2f));
  os.write((const char *)&(camera.erpThetaRange), sizeof(Vec2f));
  os.write((const char *)&(camera.cubicMapType), sizeof(CubicMapType));
  os.write((const char *)&(camera.perspectiveFocal), sizeof(Vec2f));
  os.write((const char *)&(camera.perspectiveCenter), sizeof(Vec2f));
  os.write((const char *)&(camera.depthRange), sizeof(Vec2f));
}

void writeCameraListToFile(std::ofstream &os, const CameraParameterList &list) {
  uint16_t nbCamera = uint16_t(list.size());

  os.write((char *)&nbCamera, sizeof(uint16_t));

  for (const auto &camera : list)
    writeCameraToFile(os, camera);
}

std::vector<Vec2i> readAtlasSizeFromFile(std::ifstream &is) {
  uint8_t nbAtlas = 0;
  std::vector<Vec2i> result;

  is.read((char *)&nbAtlas, sizeof(uint8_t));

  result.resize(nbAtlas);
  is.read((char *)result.data(), nbAtlas * sizeof(Vec2i));

  return result;
}

void skipAtlasSizeFromFile(std::ifstream &is) {
  uint8_t nbAtlas = 0;

  is.read((char *)&nbAtlas, sizeof(uint8_t));

  is.seekg(nbAtlas * sizeof(Vec2i), std::ifstream::cur);
}

void writeAtlasSizeToFile(std::ofstream &os,
                          const std::vector<Vec2i> &atlasSize) {
  uint8_t nbAtlas = uint8_t(atlasSize.size());

  os.write((const char *)&nbAtlas, sizeof(uint8_t));

  os.write((const char *)atlasSize.data(), nbAtlas * sizeof(Vec2i));
}

PatchParameters readPatchFromFile(std::ifstream &is) {
  PatchParameters patch;

  is.read((char *)&(patch.atlasId), sizeof(uint8_t));
  is.read((char *)&(patch.virtualCameraId), sizeof(uint8_t));
  is.read((char *)&(patch.patchSize), sizeof(Vec2i));
  is.read((char *)&(patch.patchMappingPos), sizeof(Vec2i));
  is.read((char *)&(patch.patchPackingPos), sizeof(Vec2i));
  is.read((char *)&(patch.patchRotation), sizeof(PatchRotation));

  return patch;
}

PatchParameterList readPatchListFromFile(std::ifstream &is) {
  uint16_t nbPatch = 0;
  PatchParameterList list;

  is.read((char *)&nbPatch, sizeof(uint16_t));

  for (auto i = 0; i < nbPatch; i++)
    list.push_back(readPatchFromFile(is));

  return list;
}

void skipPatchListFromFile(std::istream &is) {
  size_t patchSizeInFile = sizeof(uint8_t) + sizeof(uint8_t) + sizeof(Vec2i) +
                           sizeof(Vec2i) + sizeof(Vec2i) +
                           sizeof(PatchRotation);

  uint16_t nbPatch = 0;

  is.read((char *)&nbPatch, sizeof(uint16_t));

  is.seekg(nbPatch * patchSizeInFile, std::ifstream::cur);
}

void writePatchToFile(std::ofstream &os, const PatchParameters &patch) {
  os.write((const char *)&(patch.atlasId), sizeof(uint8_t));
  os.write((const char *)&(patch.virtualCameraId), sizeof(uint8_t));
  os.write((const char *)&(patch.patchSize), sizeof(Vec2i));
  os.write((const char *)&(patch.patchMappingPos), sizeof(Vec2i));
  os.write((const char *)&(patch.patchPackingPos), sizeof(Vec2i));
  os.write((const char *)&(patch.patchRotation), sizeof(PatchRotation));
}

void writePatchListToFile(std::ofstream &os, const PatchParameterList &list) {
  uint16_t nbPatch = uint16_t(list.size());

  os.write((char *)&nbPatch, sizeof(uint16_t));

  for (const auto &patch : list)
    writePatchToFile(os, patch);
}

template <typename T>
T readMetadataFromFile(const std::string &path, int frameIndex,
                       std::function<void(std::ifstream &)> skipFunction,
                       std::function<T(std::ifstream &)> readFunction) {
  std::ifstream stream{path, std::ios::binary};

  if (!stream.good())
    throw runtime_error("Failed to open file: " + path);

  // Seeking
  stream.seekg(streamoff(0), ifstream::beg);

  while (true) {
    std::uint32_t frameId = 0;
    stream.read((char *)&frameId, sizeof(std::uint32_t));

    if (!stream.good())
      throw runtime_error("Failed to read frame #" +
                          std::to_string(frameIndex) + " from file: " + path);

    if (frameId != std::uint32_t(frameIndex))
      skipFunction(stream);
    else
      break;
  }

  // Reading
  return readFunction(stream);
}

template <typename T>
void writeMetadataToFile(
    const std::string &path, int frameIndex, const T &metadata,
    std::function<void(std::ofstream &, const T &)> writeFunction, bool trunc) {
  std::ofstream stream{path,
                       (trunc ? std::ofstream::trunc : std::ofstream::app) |
                           std::ofstream::binary};

  if (!stream.good())
    throw runtime_error("Failed to open file: " + path);

  // Frame index
  std::uint32_t frameId = frameIndex;
  stream.write((const char *)&frameId, sizeof(std::uint32_t));

  if (!stream.good())
    throw runtime_error("Failed to write frame #" + std::to_string(frameIndex) +
                        " to file: " + path);

  // Metadata
  writeFunction(stream, metadata);
}

} // namespace

/////////////////////////////////////////////////
CameraParameterList loadSourceMetadata(const Json &config) {
  cout << "Loading source metadata\n";

  std::string cameraPath =
      getFullPath(config, "SourceDirectory", "SourceCameraParameters");
  ifstream stream{cameraPath};

  if (!stream.good())
    throw runtime_error("Failed to load source camera parameters");

  auto cameras =
      loadCamerasFromJson(Json{stream}.require("cameras"),
                          config.require("SourceCameraNames").asStringVector());

  for (const auto &camera : cameras)
    cout << camera << '\n';

  return cameras;
}

MVD16Frame loadSourceFrame(const Json &config,
                           const CameraParameterList &cameras, int frameIndex) {
  cout << "Loading source frame " << frameIndex << '\n';

  MVD16Frame result;

  frameIndex += config.require("startFrame").asInt();
  for (const auto &cam : cameras) {
    std::string texturePath =
        getFullPath(config, "SourceDirectory", "SourceTexturePathFmt", cam.id);
    auto textureFrame = readFrame<YUV420P10>(texturePath, frameIndex, cam.size);

    std::string depthPath =
        getFullPath(config, "SourceDirectory", "SourceDepthPathFmt", cam.id);
    auto depthFrame = readFrame<YUV400P16>(depthPath, frameIndex, cam.size);

    result.push_back(
        TextureDepth16Frame(std::move(textureFrame), std::move(depthFrame)));
  }

  return result;
}

/////////////////////////////////////////////////
BaseAdditional<CameraParameterList> loadOptimizedMetadata(const Json &config,
                                                          int frameIndex) {
  cout << "Loading optimized metadata\n";

  BaseAdditional<CameraParameterList> result;
  std::string baseMetadataPath =
      getFullPath(config, "OutputDirectory", "BaseMetadataPath");
  std::string additionalMetadataPath =
      getFullPath(config, "OutputDirectory", "AdditionalMetadataPath");

  auto skipFunction = [](std::ifstream &is) { skipCameraListFromFile(is); };

  auto readFunction = [](std::ifstream &is) -> CameraParameterList {
    return readCameraListFromFile(is);
  };

  // Reading
  return BaseAdditional<CameraParameterList>{
      readMetadataFromFile<CameraParameterList>(baseMetadataPath, frameIndex,
                                                skipFunction, readFunction),
      readMetadataFromFile<CameraParameterList>(
          additionalMetadataPath, frameIndex, skipFunction, readFunction)};
}

void saveOptimizedMetadata(
    const Json &config, int frameIndex,
    const BaseAdditional<CameraParameterList> &metadata) {
  cout << "Saving metadata of optimized frame " << frameIndex << '\n';

  std::string baseMetadataPath =
      getFullPath(config, "OutputDirectory", "BaseMetadataPath");
  std::string additionalMetadataPath =
      getFullPath(config, "OutputDirectory", "AdditionalMetadataPath");

  auto writeFunction = [](std::ofstream &os,
                          const CameraParameterList &metadata) {
    writeCameraListToFile(os, metadata);
  };

  writeMetadataToFile<CameraParameterList>(baseMetadataPath, frameIndex,
                                           metadata.base, writeFunction,
                                           (frameIndex == 0));
  writeMetadataToFile<CameraParameterList>(additionalMetadataPath, frameIndex,
                                           metadata.additional, writeFunction,
                                           (frameIndex == 0));
}

BaseAdditional<MVD16Frame>
loadOptimizedFrame(const Json &config,
                   const BaseAdditional<Metadata::CameraParameterList> &cameras,
                   int frameIndex) {
  cout << "Loading optimized frame " << frameIndex << '\n';

  BaseAdditional<MVD16Frame> frame;

  for (const auto &cam : cameras.base) {
    std::string texturePath =
        getFullPath(config, "OutputDirectory", "BaseTexturePathFmt", cam.id);
    auto textureFrame = readFrame<YUV420P10>(texturePath, frameIndex, cam.size);

    std::string depthPath =
        getFullPath(config, "OutputDirectory", "BaseDepthPathFmt", cam.id);
    auto depthFrame = readFrame<YUV400P16>(depthPath, frameIndex, cam.size);

    frame.base.push_back(
        TextureDepth16Frame(std::move(textureFrame), std::move(depthFrame)));
  }

  for (const auto &cam : cameras.additional) {
    std::string texturePath = getFullPath(config, "OutputDirectory",
                                          "AdditionalTexturePathFmt", cam.id);
    auto textureFrame = readFrame<YUV420P10>(texturePath, frameIndex, cam.size);

    std::string depthPath = getFullPath(config, "OutputDirectory",
                                        "AdditionalDepthPathFmt", cam.id);
    auto depthFrame = readFrame<YUV400P16>(depthPath, frameIndex, cam.size);

    frame.base.push_back(
        TextureDepth16Frame(std::move(textureFrame), std::move(depthFrame)));
  }

  return frame;
}

void saveOptimizedFrame(const Json &config, int frameIndex,
                        const BaseAdditional<CameraParameterList> &cameras,
                        const BaseAdditional<MVD16Frame> &frame) {
  cout << "Saving optimized frame " << frameIndex << '\n';

  assert(cameras.base.size() == frame.base.size());

  for (auto i = 0u; i < cameras.base.size(); i++) {
    std::string texturePath = getFullPath(
        config, "OutputDirectory", "BaseTexturePathFmt", cameras.base[i].id);
    writeFrame<YUV420P10>(texturePath, frame.base[i].first, (frameIndex == 0));

    std::string depthPath = getFullPath(config, "OutputDirectory",
                                        "BaseDepthPathFmt", cameras.base[i].id);
    writeFrame<YUV400P16>(depthPath, frame.base[i].second, (frameIndex == 0));
  }

  assert(cameras.additional.size() == frame.additional.size());

  for (auto i = 0u; i < cameras.additional.size(); i++) {
    std::string texturePath =
        getFullPath(config, "OutputDirectory", "AdditionalTexturePathFmt",
                    cameras.additional[i].id);
    writeFrame<YUV420P10>(texturePath, frame.additional[i].first,
                          (frameIndex == 0));

    std::string depthPath =
        getFullPath(config, "OutputDirectory", "AdditionalDepthPathFmt",
                    cameras.additional[i].id);
    writeFrame<YUV400P16>(depthPath, frame.additional[i].second,
                          (frameIndex == 0));
  }
}

/////////////////////////////////////////////////
MivMetadata loadMivMetadata(const Json &config, int frameIndex) {
  cout << "Loading MIV metadata of frame " << frameIndex << '\n';

  MivMetadata result;
  std::string metadataPath =
      getFullPath(config, "OutputDirectory", "AtlasMetadataPath");

  auto skipFunction = [](std::ifstream &is) {
    skipAtlasSizeFromFile(is);
    skipPatchListFromFile(is);
    skipCameraListFromFile(is);
  };

  auto readFunction = [](std::ifstream &is) -> MivMetadata {
    auto atlasSize = readAtlasSizeFromFile(is);
    auto patchList = readPatchListFromFile(is);
    auto cameraList = readCameraListFromFile(is);

    return MivMetadata{std::move(atlasSize), std::move(patchList),
                       std::move(cameraList)};
  };

  // Reading
  return readMetadataFromFile<MivMetadata>(metadataPath, frameIndex,
                                           skipFunction, readFunction);
}

void saveMivMetadata(const Json &config, int frameIndex,
                     const MivMetadata &metadata) {
  cout << "Saving MIV metadata of frame " << frameIndex << '\n';

  std::string metadataPath =
      getFullPath(config, "OutputDirectory", "AtlasMetadataPath");

  auto writeFunction = [](std::ofstream &os, const MivMetadata &metadata) {
    writeAtlasSizeToFile(os, metadata.atlasSize);
    writePatchListToFile(os, metadata.patches);
    writeCameraListToFile(os, metadata.cameras);
  };

  writeMetadataToFile<MivMetadata>(metadataPath, frameIndex, metadata,
                                   writeFunction, (frameIndex == 0));
}

MVD10Frame loadAtlas(const Json &config,
                     const std::vector<Common::Vec2i> &atlasSize,
                     int frameIndex) {
  cout << "Loading atlas frame " << frameIndex << '\n';

  MVD10Frame result;

  for (auto id = 0u; id < atlasSize.size(); id++) {
    std::string texturePath =
        getFullPath(config, "OutputDirectory", "AtlasTexturePathFmt", id);
    auto textureFrame =
        readFrame<YUV420P10>(texturePath, frameIndex, atlasSize[id]);

    std::string depthPath =
        getFullPath(config, "OutputDirectory", "AtlasDepthPathFmt", id);
    auto depthFrame =
        readFrame<YUV420P10>(depthPath, frameIndex, atlasSize[id]);

    Frame<YUV400P10> depth10(depthFrame.getWidth(), depthFrame.getHeight());
    convert(depthFrame, depth10);

    result.push_back(
        TextureDepth10Frame(std::move(textureFrame), std::move(depth10)));
  }

  return result;
}

void saveAtlas(const Json &config, int frameIndex, const MVD16Frame &frame) {
  cout << "Saving atlas frame " << frameIndex << '\n';

  Frame<YUV420P10> depth10;

  for (auto id = 0u; id < frame.size(); id++) {
    std::string texturePath =
        getFullPath(config, "OutputDirectory", "AtlasTexturePathFmt", id);
    writeFrame<YUV420P10>(texturePath, frame[id].first, (frameIndex == 0));

    depth10.resize(frame[id].second.getWidth(), frame[id].second.getHeight());
    convert(frame[id].second, depth10);

    std::string depthPath =
        getFullPath(config, "OutputDirectory", "AtlasDepthPathFmt", id);
    writeFrame<YUV420P10>(depthPath, depth10, (frameIndex == 0));
  }
}

/////////////////////////////////////////////////
PatchIdMapList loadPatchIdMaps(const Json &config,
                               const std::vector<Common::Vec2i> &atlasSize,
                               int frameIndex) {
  cout << "Loading patchIdMap frame " << frameIndex << '\n';

  PatchIdMapList result;

  for (auto id = 0u; id < atlasSize.size(); id++) {
    std::string texturePath =
        getFullPath(config, "OutputDirectory", "AtlasPatchOccupancyMapFmt", id);
    auto textureFrame =
        readFrame<YUV400P16>(texturePath, frameIndex, atlasSize[id]);

    result.push_back(std::move(textureFrame));
  }

  return result;
}

void savePatchIdMaps(const Json &config, int frameIndex,
                     const PatchIdMapList &maps) {
  cout << "Saving patchIdMap frame " << frameIndex << '\n';

  for (auto id = 0u; id < maps.size(); id++) {
    std::string texturePath =
        getFullPath(config, "OutputDirectory", "AtlasPatchOccupancyMapFmt", id);
    writeFrame<YUV400P16>(texturePath, maps[id], (frameIndex == 0));
  }
}

/////////////////////////////////////////////////
CameraParameters loadViewportMetadata(const Json &config, int frameIndex) {
  // TODO
  return {};
}

void saveViewport(const Json &config, int frameIndex,
                  const TextureDepth10Frame &frame) {
  cout << "Saving viewport frame " << frameIndex << '\n';

  std::string texturePath =
      getFullPath(config, "OutputDirectory", "RenderedTexturePath", frameIndex);
  writeFrame<YUV420P10>(texturePath, frame.first, (frameIndex == 0));

  std::string depthPath =
      getFullPath(config, "OutputDirectory", "RenderedDepthPath", frameIndex);
  writeFrame<YUV400P10>(depthPath, frame.second, (frameIndex == 0));
}

} // namespace TMIV::IO
