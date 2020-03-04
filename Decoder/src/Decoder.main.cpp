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
#include <TMIV/Metadata/ViewingSpace.h>

#include <iostream>
#include <memory>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Decoder {
class Application : public Common::Application {
private:
  unique_ptr<IDecoder> m_decoder;
  int m_numberOfFrames;
  int m_intraPeriod;
  IO::IvMetadataReader m_metadataReader;

public:
  explicit Application(vector<const char *> argv)
      : Common::Application{"Decoder", move(argv)}, m_metadataReader{json(), "OutputDirectory",
                                                                     "AtlasMetadataPath"} {
    m_decoder = create<IDecoder>("Decoder");
    m_numberOfFrames = json().require("numberOfFrames").asInt();
    m_intraPeriod = json().require("intraPeriod").asInt();

    if (auto subnode = json().optional("extraNumberOfFrames")) {
      m_numberOfFrames += subnode.asInt();
    }
  }

  void run() override {
    m_metadataReader.readIvSequenceParams();
    auto ivSequenceParams = m_metadataReader.ivSequenceParams();
    {
      if (auto subnode = json().optional("ViewingSpace"); subnode) {
        cout << "Overriding viewing space from JSON" << endl;
        ivSequenceParams.viewingSpace = Metadata::ViewingSpace::loadFromJson(subnode);
      }
    }
    m_decoder->updateSequenceParams(ivSequenceParams);
    cout << "Decoded sequence parameters:\n" << ivSequenceParams;

    int firstOutputFrame = 0;
    int outputFrameStep = 1;
    if (auto subnode = json().optional("firstOutputFrame"); subnode) {
      firstOutputFrame = subnode.asInt();
    }
    if (auto subnode = json().optional("outputFrameStep"); subnode) {
      outputFrameStep = subnode.asInt();
    }
    for (int outputFrame = firstOutputFrame; outputFrame < m_numberOfFrames;
         outputFrame += outputFrameStep) {
      auto inputFrame = IO::getExtendedIndex(json(), outputFrame);
      cout << "\nDECODE INPUT FRAME " << inputFrame << " TO OUTPUT FRAME " << outputFrame
           << ":\n\n";

      if (m_metadataReader.readAccessUnit(inputFrame / m_intraPeriod)) {
        cout << "\nDecoded access unit parameters:\n" << m_metadataReader.ivAccessUnitParams();
        m_decoder->updateAccessUnitParams(m_metadataReader.ivAccessUnitParams());
      }

      const auto viewportParams = IO::loadViewportMetadata(json(), outputFrame);
      cout << "Target viewport: " << viewportParams << "\n";

      const auto &atlasSizes = m_metadataReader.ivAccessUnitParams().atlasParamsList->atlasSizes;
      const auto viewport =
          m_decoder->decodeFrame(IO::loadAtlas(json(), atlasSizes, inputFrame), viewportParams);
      IO::saveViewport(json(), outputFrame, {yuv420p(viewport.first), viewport.second});

      //////////////////////////////////////////////////////////////////////////////////////
      // dumping intermediate results to disk
      if (auto subnode = json().optional("AtlasPatchOccupancyMapFmt")) {
        std::cout << "Dumping patch map Id list to disk" << std::endl;
        auto patchMapIdList =
            m_decoder->getPatchIdMapList();
        IO::savePatchIdMaps(json(), outputFrame, patchMapIdList);
      };
      if (auto subnode1 = json().optional("PrunedViewTexturePathFmt")) {
        if (auto subnode2 = json().optional("PrunedViewDepthPathFmt")) {
          std::cout << "Dumping recovered pruned views to disk" << std::endl;
          auto recoveredTransportView =
              m_decoder->recoverPrunedView(IO::loadAtlas(json(), atlasSizes, inputFrame));
          IO::savePrunedFrame(json(), outputFrame, recoveredTransportView);
        }
      }
      //////////////////////////////////////////////////////////////////////////////////////
    }
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
