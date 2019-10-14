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

#include <TMIV/AtlasDeconstructor/IAtlasDeconstructor.h>
#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/IO/IO.h>
#include <TMIV/IO/IvMetadataReader.h>
#include <TMIV/Metadata/Bitstream.h>
#include <iostream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::AtlasDeconstructor {
class Application : public Common::Application {
private:
  unique_ptr<IAtlasDeconstructor> m_atlasDeconstructor;
  int m_numberOfFrames{};
  int m_intraPeriod{};
  IO::IvMetadataReader m_metadataReader;

public:
  explicit Application(vector<const char *> argv)
      : Common::Application{"AtlasDeconstructor", move(argv)},
        m_atlasDeconstructor{create<IAtlasDeconstructor>("Decoder", "AtlasDeconstructor")},
        m_numberOfFrames{json().require("numberOfFrames").asInt()},
        m_intraPeriod{json().require("intraPeriod").asInt()}, m_metadataReader{
                                                                  json(), "OutputDirectory",
                                                                  "AtlasMetadataPath"} {}

  void run() override {
    m_metadataReader.readIvSequenceParams();
    cout << "Camera parameters:\n" << m_metadataReader.cameraParamsList();

    for (int i = 0; i < m_numberOfFrames; i += m_intraPeriod) {
      int endFrame = min(m_numberOfFrames, i + m_intraPeriod);
      cout << "Intra period: [" << i << ", " << endFrame << ")\n";
      runIntraPeriod(i, endFrame);
    }
  }

  void runIntraPeriod(int intraFrame, int endFrame) {
    m_metadataReader.readIvAccessUnitParams();

    cout << "OMAF v1 compatible flag: " << boolalpha << m_metadataReader.omafV1CompatibleFlag()
         << endl;
    printStatistics();

    for (int i = intraFrame; i < endFrame; ++i) {
      auto atlas = IO::loadAtlas(json(), m_metadataReader.atlasSizes(), i);

      auto patchIdMaps = m_atlasDeconstructor->getPatchIdMap(
          m_metadataReader.atlasSizes(), m_metadataReader.atlasParamsList(),
          m_metadataReader.cameraParamsList(), atlas);
      IO::savePatchIdMaps(json(), i, patchIdMaps);

      auto recoveredTransportView = m_atlasDeconstructor->recoverPrunedView(
          atlas, m_metadataReader.cameraParamsList(), m_metadataReader.atlasParamsList());
      IO::savePrunedFrame(json(), i, recoveredTransportView);
    }
  }

private:
  void printStatistics() const {
    map<size_t, size_t> numPatchesPerAtlas;
    map<size_t, size_t> numPatchesPerView;

    for (const auto &patch : m_metadataReader.atlasParamsList()) {
      ++numPatchesPerAtlas.insert({patch.atlasId, 0}).first->second;
      ++numPatchesPerView.insert({patch.viewId, 0}).first->second;
    }

    for (auto [atlasId, numPatches] : numPatchesPerAtlas) {
      auto atlasSize = m_metadataReader.atlasSizes()[atlasId];
      cout << "Atlas #" << atlasId << " (" << atlasSize.x() << "x" << atlasSize.y()
           << "): " << numPatches << " patches\n";
    }

    for (auto [viewId, numPatches] : numPatchesPerView) {
      cout << "View #" << viewId << ": " << numPatches << " patches\n";
    }
  }
};

} // namespace TMIV::AtlasDeconstructor

#include "AtlasDeconstructor.reg.hpp"

int main(int argc, char *argv[]) {
  try {
    TMIV::AtlasDeconstructor::registerComponents();
    TMIV::AtlasDeconstructor::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (runtime_error &e) {
    cerr << e.what() << endl;
    return 1;
  }
}
