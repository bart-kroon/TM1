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

#include <TMIV/Renderer/IRenderer.h>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/IO/IO.h>
#include <TMIV/IO/IvMetadataReader.h>

#include <iostream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Renderer {
class Application : public Common::Application {
private:
  unique_ptr<IRenderer> m_renderer;
  int m_numberOfFrames;
  int m_intraPeriod;
  IO::IvMetadataReader m_metadataReader;

public:
  explicit Application(vector<const char *> argv)
      : Common::Application{"Renderer", move(argv)}, m_metadataReader{json(), "OutputDirectory",
                                                                      "AtlasMetadataPath"} {
    m_renderer = create<IRenderer>("Decoder", "Renderer");
    m_numberOfFrames = json().require("numberOfFrames").asInt();
    m_intraPeriod = json().require("intraPeriod").asInt();

    if (auto subnode = json().optional("extraNumberOfFrames")) {
      m_numberOfFrames += subnode.asInt();
    }
  }

  void run() override {
    for (int outputFrame = 0; outputFrame < m_numberOfFrames; ++outputFrame) {
      auto inputFrame = IO::getExtendedIndex(json(), outputFrame);

      if (m_metadataReader.readAccessUnit(inputFrame / m_intraPeriod)) {
        cout << "OMAF v1 compatible flag: " << boolalpha << m_metadataReader.omafV1CompatibleFlag()
             << '\n';
      }

      auto frame = IO::loadAtlas(json(), m_metadataReader.atlasSizes(), inputFrame);
      auto patchIds = IO::loadPatchIdMaps(json(), m_metadataReader.atlasSizes(), inputFrame);
      auto target = IO::loadViewportMetadata(json(), inputFrame);
      auto viewport = m_renderer->renderFrame(frame, patchIds, m_metadataReader.atlasParamsList(),
                                              m_metadataReader.cameraParamsList(), target);
      IO::saveViewport(json(), outputFrame, {yuv420p(viewport.first), viewport.second});
    }
  }
};
} // namespace TMIV::Renderer

#include "Renderer.reg.hpp"

int main(int argc, char *argv[]) {
  try {
    TMIV::Renderer::registerComponents();
    TMIV::Renderer::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (runtime_error &e) {
    cerr << e.what() << endl;
    return 1;
  }
}
