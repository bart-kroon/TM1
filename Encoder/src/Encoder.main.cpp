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

#include <algorithm>
#include <iostream>
#include <memory>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Encoder/IEncoder.h>
#include <TMIV/IO/IO.h>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Encoder {
class Application : public Common::Application {
private:
  unique_ptr<IEncoder> m_encoder;
  int m_numberOfFrames{};
  int m_intraPeriod{};
  bool m_omafV1CompatibleFlag{};
  Metadata::CameraParametersList m_cameras;

public:
  explicit Application(vector<const char *> argv)
      : Common::Application{"Encoder", move(argv)}, m_encoder{create<IEncoder>("Encoder")},
        m_numberOfFrames{json().require("numberOfFrames").asInt()},
        m_intraPeriod{json().require("intraPeriod").asInt()},
        m_omafV1CompatibleFlag{json().require("OmafV1CompatibleFlag").asBool()} {}

  void run() override {
    m_cameras = IO::loadSourceMetadata(json());

    for (int i = 0; i < m_numberOfFrames; i += m_intraPeriod) {
      int endFrame = min(m_numberOfFrames, i + m_intraPeriod);
      cout << "Intra period: [" << i << ", " << endFrame << ")\n";
      runIntraPeriod(i, endFrame);
    }
  }

private:
  void runIntraPeriod(int intraFrame, int endFrame) {
    m_encoder->prepareIntraPeriod(m_cameras);

    for (int i = intraFrame; i < endFrame; ++i) {
      auto frame = IO::loadSourceFrame(json(), IO::sizesOf(m_cameras), i);
      m_encoder->pushFrame(move(frame));
    }

    m_encoder->completeIntraPeriod();

    IO::saveMivMetadata(json(), intraFrame,
                        {m_encoder->getAtlasSize(), m_omafV1CompatibleFlag,
                         m_encoder->getPatchList(), m_encoder->getCameraList()});

    for (int i = intraFrame; i < endFrame; ++i) {
      auto frame = m_encoder->popAtlas();
      IO::saveAtlas(json(), i, frame);
    }
  }
};
} // namespace TMIV::Encoder

#include "Encoder.reg.hpp"

int main(int argc, char *argv[]) {
  try {
    TMIV::Encoder::registerComponents();
    TMIV::Encoder::Application app{{argv, argv + argc}};
    app.startTime();
    app.run();
    app.printTime();
    return 0;
  } catch (runtime_error &e) {
    cerr << e.what() << endl;
    return 1;
  }
}
