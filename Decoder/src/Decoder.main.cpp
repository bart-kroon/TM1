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
#include <TMIV/Renderer/mapInputToOutputFrames.h>
#include <TMIV/IO/IO.h>
#include <TMIV/MivBitstream/SequenceConfig.h>
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
  MivBitstream::SequenceConfig m_sequenceConfig;

public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"Decoder", std::move(argv)}
      , m_metadataReader{json()}
      , m_decoder{create<IDecoder>("Decoder")}
      , m_inputToOutputFrameIdMap{Renderer::mapInputToOutputFrames(json())} {}

  void run() override {
    while (auto frame = m_metadataReader.decoder()()) {
      // Check which frames to render if we would
      const auto range = m_inputToOutputFrameIdMap.equal_range(frame->foc);
      if (range.first == range.second) {
        return;
      }

      // Recover geometry, occupancy, and filter blockToPatchMap
      m_decoder->recoverFrame(*frame);

      // Output reconstructed content configuration if changed
      if (json().optional("OutputSequenceConfigPathFmt")) {
        outputSequenceConfig(frame->sequenceConfig(), frame->foc);
      }

      // Output block to patch map
      if (json().optional("AtlasPatchOccupancyMapFmt")) {
        std::cout << "Dumping patch ID maps to disk" << std::endl;
        IO::saveBlockToPatchMaps(json(), frame->foc, *frame);
      }

      // Output pruned frames
      if (json().optional("PrunedViewAttributePathFmt") ||
          json().optional("PrunedViewGeometryPathFmt") ||
          json().optional("PrunedViewMaskPathFmt")) {
        std::cout << "Dumping recovered pruned views to disk" << std::endl;
        IO::savePrunedFrame(json(), frame->foc, Renderer::recoverPrunedViewAndMask(*frame));
      }

      // Render multiple frames
      if (json().optional("OutputCameraName")) {
        for (auto i = range.first; i != range.second; ++i) {
          renderDecodedFrame(*frame, i->second);
        }
      }
    }
  }

private:
  void renderDecodedFrame(AccessUnit frame, int outputFrameId) {
    std::cout << "Rendering input frame " << frame.foc << " to output frame " << outputFrameId
              << ", with target viewport:\n";
    const auto viewportParams = IO::loadViewportMetadata(json(), outputFrameId);
    viewportParams.printTo(std::cout, 0);
    
    const auto viewport = m_decoder->renderFrame(frame, viewportParams);
    IO::saveViewport(json(), outputFrameId, {yuv420p(viewport.first), viewport.second});
  }

  void outputSequenceConfig(MivBitstream::SequenceConfig sc, std::int32_t foc) {
    if (m_sequenceConfig != sc) {
      m_sequenceConfig = std::move(sc);
      if (json().optional("OutputSequenceConfigPathFmt")) {
        const auto path =
            IO::getFullPath(json(), "OutputDirectory", "OutputSequenceConfigPathFmt", foc);
        std::cout << "Writing reconstructed sequence configuration for frame " << foc << " to disk"
                  << std::endl;
        std::ofstream stream{path};
        const auto json = Common::Json{m_sequenceConfig};
        json.saveTo(stream);
      }
    }
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
  } catch (std::logic_error &e) {
    std::cerr << e.what() << std::endl;
    return 3;
  }
}
