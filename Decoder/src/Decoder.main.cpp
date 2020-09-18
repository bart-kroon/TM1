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

#include <TMIV/Decoder/IDecoder.h>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/ViewingSpace.h>
#include <TMIV/Renderer/RecoverPrunedViews.h>

#include "IvMetadataReader.h"

#include <iostream>
#include <map>
#include <memory>

namespace TMIV::Decoder {
void registerComponents();

class Application : public Common::Application {
private:
  IvMetadataReader m_metadataReader;
  std::unique_ptr<IDecoder> m_decoder;
  std::multimap<int, int> m_inputToOutputFrameIdMap;

public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"Decoder", std::move(argv)}
      , m_metadataReader{json()}
      , m_decoder{create<IDecoder>("Decoder")}
      , m_inputToOutputFrameIdMap{mapInputToOutputFrames(json())} {}

  void run() override {
    while (auto frame = m_metadataReader.decoder()()) {
      auto range = m_inputToOutputFrameIdMap.equal_range(frame->foc);
      if (range.first == range.second) {
        return; // TODO(BK): Test with A97 pose trace, then remove this comment
      }
      for (auto i = range.first; i != range.second; ++i) {
        renderDecodedFrame(*frame, i->second);
      }
    }
  }

private:
  void renderDecodedFrame(AccessUnit frame, int outputFrameId) {
    std::cout << "Rendering input frame " << frame.foc << " to output frame " << outputFrameId
              << ", with target viewport:\n";
    const auto viewportParams = IO::loadViewportMetadata(json(), outputFrameId);
    viewportParams.printTo(std::cout, 0);

    const auto viewport = m_decoder->decodeFrame(frame, viewportParams);

    IO::saveViewport(json(), outputFrameId, {yuv420p(viewport.first), viewport.second});

    if (json().optional("AtlasPatchOccupancyMapFmt")) {
      std::cout << "Dumping patch ID maps to disk" << std::endl;
      IO::saveBlockToPatchMaps(json(), outputFrameId, frame);
    }

    if (json().optional("PrunedViewAttributePathFmt")   // format: yuv444p10
        || json().optional("PrunedViewGeometryPathFmt") // format: yuv420p10
        || json().optional("PrunedViewMaskPathFmt")) {  // format: yuv420p
      std::cout << "Dumping recovered pruned views to disk" << std::endl;
      IO::savePrunedFrame(json(), outputFrameId, Renderer::recoverPrunedViewAndMask(frame));
    }
  }

  // Returns a frame index. If frameIndex is strictly less than the actual number of frames in the
  // encoded stream, then regular values are returned else mirrored indices are computed.
  static auto getExtendedIndex(const Common::Json &config, int frameIndex) -> int {
    int numberOfFrames = config.require("numberOfFrames").asInt();
    int frameGroupId = frameIndex / numberOfFrames;
    int frameRelativeId = frameIndex % numberOfFrames;
    return (frameGroupId % 2) != 0 ? (numberOfFrames - (frameRelativeId + 1)) : frameRelativeId;
  }

  static auto mapInputToOutputFrames(const Common::Json &config) -> std::multimap<int, int> {
    auto x = std::multimap<int, int>{};

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

    for (int outputFrame = firstOutputFrame; outputFrame < numberOfFrames + extraFrames;
         outputFrame += outputFrameStep) {
      const auto inputFrame = getExtendedIndex(config, outputFrame);
      x.emplace(inputFrame, outputFrame);
    }

    return x;
  }
};
} // namespace TMIV::Decoder

auto main(int argc, char *argv[]) -> int {
  try {
    TMIV::Decoder::registerComponents();
    TMIV::Decoder::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}
