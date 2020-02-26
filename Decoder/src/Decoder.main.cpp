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

#include <TMIV/Decoder/IDecoder.h>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/IO/IO.h>
#include <TMIV/IO/IvMetadataReader.h>
#include <TMIV/MivBitstream/MivDecoder.h>
#include <TMIV/MivBitstream/ViewingSpace.h>

#include <iostream>
#include <map>
#include <memory>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::MivBitstream {
const MivDecoder::Mode MivDecoder::mode = MivDecoder::Mode::MIV;
}

namespace TMIV::Decoder {
class Application : public Common::Application {
private:
  IO::IvMetadataReader m_metadataReader;
  unique_ptr<IDecoder> m_decoder;
  multimap<int, int> m_inputToOutputFramesMap;

public:
  explicit Application(vector<const char *> argv)
      : Common::Application{"Decoder", move(argv)},                       // Load configuration
        m_metadataReader{json(), "OutputDirectory", "AtlasMetadataPath"}, // Read MIV bitstream
        m_decoder{create<IDecoder>("Decoder")},                           // Decoder/renderer
        m_inputToOutputFramesMap{mapInputToOutputFrames(json())} {        // Handle pose traces

    m_metadataReader.decoder().onSequence.emplace_back(
        [this](const VpccParameterSet & /* vps */) { cout << "New sequence\n"; });

    m_metadataReader.decoder().onFrame.emplace_back([this](const AccessUnit &au) {
      auto outputFrames = m_inputToOutputFramesMap.equal_range(au.frameId);

      for (auto outputFrame = outputFrames.first; outputFrame != outputFrames.second;
           ++outputFrame) {

        renderDecodedFrame(au, outputFrame->second);
      }
      return true;
    });
  }

  void run() override { m_metadataReader.decoder().decode(); }

private:
  void renderDecodedFrame(const AccessUnit &au, int outputFrame) {
    cout << "Rendering input frame " << au.frameId << " to output frame " << outputFrame
         << ", with target viewport:\n";
    const auto viewportParams = IO::loadViewportMetadata(json(), outputFrame);
    viewportParams.printTo(cout, 0);

    const auto viewport = m_decoder->decodeFrame(au, viewportParams);

    IO::saveViewport(json(), outputFrame, {yuv420p(viewport.first), viewport.second});

    if (json().optional("AtlasPatchOccupancyMapFmt")) {
      std::cout << "Dumping patch ID maps to disk" << std::endl;
      auto patchMapIdList = m_decoder->getPatchIdMapList(au);
      IO::savePatchIdMaps(json(), outputFrame, patchMapIdList);
    }

    if (json().optional("PrunedViewTexturePathFmt") && json().optional("PrunedViewDepthPathFmt")) {
      std::cout << "Dumping recovered pruned views to disk" << std::endl;
      auto recoveredTransportView = m_decoder->recoverPrunedView(au);
      IO::savePrunedFrame(json(), outputFrame, recoveredTransportView);
    }
  }

  static auto mapInputToOutputFrames(const Json &config) -> multimap<int, int> {
    auto x = multimap<int, int>{};

    const auto numberOfFrames = config.require("numberOfFrames").asInt();
    auto extraFrames = 0;
    auto firstOutputFrame = 0;
    auto outputFrameStep = 1;

    if (auto subnode = config.optional("extraNumberOfFrames"); subnode) {
      extraFrames = subnode.asInt();
    }

    if (auto subnode = config.optional("firstOutputFrame"); subnode) {
      firstOutputFrame = subnode.asInt();
    }

    if (auto subnode = config.optional("outputFrameStep"); subnode) {
      outputFrameStep = subnode.asInt();
    }

    for (int outputFrame = firstOutputFrame; outputFrame < numberOfFrames;
         outputFrame += outputFrameStep) {
      auto inputFrame = IO::getExtendedIndex(config, outputFrame);
      x.emplace(inputFrame, outputFrame);
    }

    return x;
  }
};
} // namespace TMIV::Decoder

#include "Decoder.reg.hpp"

auto main(int argc, char *argv[]) -> int {
  try {
    TMIV::Decoder::registerComponents();
    TMIV::Decoder::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (runtime_error &e) {
    cerr << e.what() << endl;
    return 1;
  }
}
