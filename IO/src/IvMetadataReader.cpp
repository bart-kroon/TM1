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

#include <TMIV/IO/IvMetadataReader.h>

#include <TMIV/IO/IO.h>

#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::IO {
IvMetadataReader::IvMetadataReader(const Json &config, const string &baseDirectoryField,
                                   const string &fileNameField)
    : m_config{config}, m_path{getFullPath(config, baseDirectoryField, fileNameField)},
      m_stream{m_path, ios::binary} {
  if (!m_stream.good()) {
    ostringstream what;
    what << "Failed to open \"" << m_path << "\" for reading";
    throw runtime_error(what.str());
  }
  m_decoder = make_unique<MivDecoder>(m_stream, geoFrameServer(), attrFrameServer());
}

auto IvMetadataReader::geoFrameServer() -> MivDecoder::GeoFrameServer {
  return [this](uint8_t atlasId, uint32_t frameId, Vec2i frameSize) {
    const auto path = getFullPath(m_config, "OutputDirectory", "AtlasDepthPathFmt", atlasId);
    cout << "Loading geometry video data: atlasId = " << int(atlasId) << ", frameId = " << frameId
         << ", frameSize = " << frameSize << " (YUV 4:2:0 --> YUV 4:0:0)\n";
    return readFrame<YUV400P10>(path, frameId, frameSize);
  };
}

auto IvMetadataReader::attrFrameServer() -> MivDecoder::AttrFrameServer {
  return [this](uint8_t atlasId, uint32_t frameId, Vec2i frameSize) {
    const auto path = getFullPath(m_config, "OutputDirectory", "AtlasTexturePathFmt", atlasId);
    cout << "Loading attribute video data: atlasId = " << int(atlasId) << ", frameId = " << frameId
         << ", frameSize = " << frameSize << " (YUV 4:2:0 --> YUV 4:4:4)\n";
    return yuv444p(readFrame<YUV420P10>(path, frameId, frameSize));
  };
}
} // namespace TMIV::IO
