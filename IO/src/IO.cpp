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

#include <fstream>
#include <iostream>

#include <TMIV/Common/Common.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::IO {
CameraParameterList loadSourceMetadata(const Json &config) {
  ifstream stream{config.require("SourceCameraParameters").asString()};
  if (!stream.good()) {
    throw runtime_error("Failed to load source camera parameters");
  }
  return loadCamerasFromJson(
      Json{stream}.require("cameras"),
      config.require("SourceCameraNames").asStringVector());
}

namespace {
TextureFrame loadSourceTexture(const Json &config, int frameIndex,
                               Vec2i resolution, size_t cameraId) {
  TextureFrame result(resolution.x(), resolution.y());
  auto path = format(config.require("SourceTexturePathFmt").asString().c_str(),
                     cameraId);
  ifstream stream{path, std::ios::binary};
  if (!stream.good()) {
    throw runtime_error("Failed to open source texture file");
  }
  stream.seekg(streampos(frameIndex) * resolution.x() * resolution.y() * 3);
  result.read(stream);
  if (!stream.good()) {
    throw runtime_error("Failed to read from source texture file");
  }
  return result;
}

Depth16Frame loadSourceDepth(const Json &config, int frameIndex,
                             Vec2i resolution, size_t cameraId) {
  Depth16Frame result(resolution.x(), resolution.y());
  auto path =
      format(config.require("SourceDepthPathFmt").asString().c_str(), cameraId);
  ifstream stream{path, std::ios::binary};
  if (!stream.good()) {
    throw runtime_error("Failed to open source texture file");
  }
  stream.seekg(streampos(frameIndex) * resolution.x() * resolution.y() * 3);
  result.read(stream);
  if (!stream.good()) {
    throw runtime_error("Failed to read from source texture file");
  }
  return result;
}
} // namespace

MVD16Frame loadSourceFrame(const Json &config, int frameIndex) {
  MVD16Frame result(config.require("SourceCameraNames").size());
  const auto resolution = config.require("SourceResolution").asIntVector<2>();
  for (size_t id = 0; id < result.size(); ++id) {
    result[id].first = loadSourceTexture(config, frameIndex, resolution, id);
    result[id].second = loadSourceDepth(config, frameIndex, resolution, id);
  }
  return result;
}

void saveOptimizedFrame(const Json &config, int frameIndex,
                        const std::pair<MVD16Frame, MVD16Frame> &frame) {}

auto loadOptimizedFrame(const Json &config, int frameIndex)
    -> std::pair<MVD16Frame, MVD16Frame> {
  return {};
}

void saveOptimizedMetadata(
    const Json &config, int frameIndex,
    const std::pair<CameraParameterList, CameraParameterList> &metadata) {}

auto loadOptimizedMetadata(const Json &config, int frameIndex)
    -> std::pair<CameraParameterList, CameraParameterList> {
  return {};
}

void saveMivMetadata(const Json &config, int frameIndex,
                     const MivMetadata &metadata) {}

auto loadMivMetadata(const Json &config, int frameIndex) -> MivMetadata {
  return {};
}

namespace {
void saveAtlasTexture(const Json &config, int frameIndex,
                      TextureFrame const &frame, size_t atlasId) {
  auto path = Common::format(
      config.require("AtlasTexturePathFmt").asString().c_str(), atlasId);
  ofstream stream{path, (frameIndex == 0 ? ofstream::trunc : ofstream::app) |
                            ofstream::binary};
  if (!stream.good()) {
    throw runtime_error("Failed to open atlas texture for writing");
  }
  frame.dump(stream);
  if (!stream.good()) {
    throw runtime_error("Failed to write to atlas texture");
  }
}

void saveAtlasDepth(const Json &config, int frameIndex,
                    Frame<YUV420P10> const &frame, size_t atlasId) {
  auto path = Common::format(
      config.require("AtlasDepthPathFmt").asString().c_str(), atlasId);
  ofstream stream(path, ((frameIndex == 0) ? ofstream::trunc : ofstream::app) |
                            ofstream::binary);
  if (!stream.good()) {
    throw runtime_error("Failed to open atlas depth map for writing");
  }
  frame.dump(stream);
  if (!stream.good()) {
    throw runtime_error("Failed to write to atlas depth map");
  }
}
} // namespace

void saveAtlas(const Json &config, int frameIndex, MVD16Frame const &frame) {
  Frame<YUV420P10> depth10;

  for (size_t id = 0; id < frame.size(); ++id) {
    saveAtlasTexture(config, frameIndex, frame[id].first, id);
    depth10.resize(frame[id].second.getWidth(), frame[id].second.getHeight());
    convert(frame[id].second, depth10);
    saveAtlasDepth(config, frameIndex, depth10, id);
  }
}

auto loadAtlas(const Json &config, int frameIndex) -> MVD10Frame { return {}; }

void savePatchIdMaps(const Json &config, int frameIndex,
                     const PatchIdMapList &maps) {}

auto loadPatchIdMaps(const Json &config, int frameIndex) -> PatchIdMapList {
  return {};
}

auto loadViewportMetadata(const Json &config, int frameIndex)
    -> CameraParameters {
  return {};
}

void saveViewport(const Json &config, int frameIndex,
                  const TextureDepth10Frame &frame) {}
} // namespace TMIV::IO
