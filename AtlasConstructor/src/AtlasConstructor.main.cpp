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

#include <TMIV/AtlasConstructor/IAtlasConstructor.h>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Thread.h>
#include <TMIV/IO/IO.h>
#include <TMIV/IO/IvMetadataWriter.h>
#include <TMIV/Image/Image.h>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Image;
using namespace TMIV::Metadata;

namespace TMIV::AtlasConstructor {
class Application : public Common::Application {
private:
  unique_ptr<IAtlasConstructor> m_atlasConstructor;
  int m_numberOfFrames{};
  int m_intraPeriod{};
  bool m_omafV1CompatibleFlag{};
  IO::BasicAdditional<ViewParamsList> m_viewParamsVector;
  IO::IvMetadataWriter m_metadataWriter;

public:
  explicit Application(vector<const char *> argv)
      : Common::Application{"AtlasConstructor", move(argv)},
        m_atlasConstructor{create<IAtlasConstructor>("Encoder", "AtlasConstructor")},
        m_numberOfFrames{json().require("numberOfFrames").asInt()},
        m_intraPeriod{json().require("intraPeriod").asInt()},
        m_omafV1CompatibleFlag{json().require("OmafV1CompatibleFlag").asBool()},
        m_metadataWriter{json(), "OutputDirectory", "AtlasMetadataPath"} {}

  void run() override {
    m_viewParamsVector = IO::loadOptimizedMetadata(json());
    cout << "Basic viewParamsVector:\n" << m_viewParamsVector.basic;
    cout << "Additional viewParamsVector:\n" << m_viewParamsVector.additional;

    for (int i = 0; i < m_numberOfFrames; i += m_intraPeriod) {
      int endFrame = min(m_numberOfFrames, i + m_intraPeriod);
      cout << "Intra period: [" << i << ", " << endFrame << ")\n";
      runIntraPeriod(i, endFrame);
    }
  }

  void runIntraPeriod(int intraFrame, int endFrame) {
    m_atlasConstructor->prepareIntraPeriod(m_viewParamsVector.basic, m_viewParamsVector.additional);

    for (int i = intraFrame; i < endFrame; ++i) {
      auto views = IO::loadOptimizedFrame(
          json(),
          {IO::sizesOf(m_viewParamsVector.basic), IO::sizesOf(m_viewParamsVector.additional)}, i);
      m_atlasConstructor->pushFrame(move(views.basic), move(views.additional));
    }

    m_atlasConstructor->completeIntraPeriod();

    auto atlasSize = m_atlasConstructor->getAtlasSize();

    for (size_t i = 0; i < atlasSize.size(); i++) {
      auto sz = atlasSize[i];
      auto nbPatch = count_if(m_atlasConstructor->getPatchList().begin(),
                              m_atlasConstructor->getPatchList().end(),
                              [i](const AtlasParameters &p) { return (p.atlasId == i); });

      cout << "Atlas #" << i << " (" << sz.x() << 'x' << sz.y() << "): " << nbPatch << " patches\n";
    }

    if (intraFrame == 0) {
      m_metadataWriter.writeIvSequenceParams(
          {{}, ViewParamsList{modifyDepthRange(m_atlasConstructor->getViewParamsVector())}});
    }
    m_metadataWriter.writeIvAccessUnitParams(
        {{{m_atlasConstructor->getPatchList(), m_omafV1CompatibleFlag, atlasSize}}});

    for (int i = intraFrame; i < endFrame; ++i) {
      IO::saveAtlas(json(), i,
                    modifyDepthRange(m_atlasConstructor->popAtlas(),
                                     m_atlasConstructor->getViewParamsVector(),
                                     m_metadataWriter.viewParamsList()));
    }
  }
};
} // namespace TMIV::AtlasConstructor

#include "AtlasConstructor.reg.hpp"

int main(int argc, char *argv[]) {
  try {
    TMIV::AtlasConstructor::registerComponents();
    TMIV::AtlasConstructor::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (runtime_error &e) {
    cerr << e.what() << endl;
    return 1;
  }
}
