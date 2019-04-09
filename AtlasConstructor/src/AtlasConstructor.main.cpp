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

#include <TMIV/AtlasConstructor/IAtlasConstructor.h>
#include <TMIV/Common/Application.h>
#include <TMIV/IO/IO.h>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;

namespace TMIV::AtlasConstructor {

class Application : public Common::Application {
private:
  unique_ptr<IAtlasConstructor> m_atlasConstructor;
  int m_numberOfFrames{};
  int m_intraPeriod{};

public:
  Application(vector<const char *> argv)
      : Common::Application{"AtlasConstructor", move(argv)},
        m_atlasConstructor{create<IAtlasConstructor>("AtlasConstructor")},
        m_numberOfFrames{json().require("numberOfFrames").asInt()},
        m_intraPeriod{json().require("intraPeriod").asInt()} {}

  void run() override {
    for (int i = 0; i < m_numberOfFrames; i += m_intraPeriod) {
      int endFrame = min(m_numberOfFrames, i + m_intraPeriod);
      cout << "Intra period: [" << i << ", " << endFrame << ")\n";
      runIntraPeriod(i, endFrame);
    }
  }

  void runIntraPeriod(int intraFrame, int endFrame) {
    auto cameras = IO::loadOptimizedMetadata(json(), intraFrame);
    m_atlasConstructor->prepareIntraPeriod(cameras.base, cameras.additional);

    for (int i = intraFrame; i < endFrame; ++i) {
      auto views = IO::loadOptimizedFrame(json(), cameras, i);
      m_atlasConstructor->pushFrame(move(views.base), move(views.additional));
    }

    m_atlasConstructor->completeIntraPeriod();

    IO::saveMivMetadata(json(), intraFrame,
                        {m_atlasConstructor->getAtlasSize(),
                         m_atlasConstructor->getPatchList(),
                         m_atlasConstructor->getCameraList()});

    for (int i = intraFrame; i < endFrame; ++i) {
      auto frame = m_atlasConstructor->popAtlas();
      IO::saveAtlas(json(), i, frame);
    }
  }
};
} // namespace TMIV::AtlasConstructor

#include "AtlasConstructor.reg.hpp"

int main(int argc, char *argv[]) {
  try {
    TMIV::AtlasConstructor::registerComponents();
    TMIV::AtlasConstructor::Application app{{argv, argv + argc}};
    app.run();
    return 0;
  } catch (runtime_error &e) {
    cerr << e.what() << endl;
  }
}