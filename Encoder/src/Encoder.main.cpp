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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Common/Json.h>
#include <TMIV/Encoder/IEncoder.h>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Encoder {
class Application : public Common::Application {
public:
  Application(vector<const char *> argv)
      : Common::Application{"Encoder", move(argv)} {
    m_encoder = create<IEncoder>("Encoder");
    m_startFrame = json().require("startFrame").asInt();
    m_numberOfFrames = json().require("numberOfFrames").asInt();
    m_intraPeriod = json().require("intraPeriod").asInt();
    m_sourceResolution = json().require("SourceResolution").asIntVector<2>();
  }

  void run() override {
    auto cameras = loadCameras();

    for (int intraFrame = 0; intraFrame < m_numberOfFrames;
         intraFrame += m_intraPeriod) {
      int endFrame = min(m_numberOfFrames, intraFrame + m_intraPeriod);
      cout << "Intra period: [" << intraFrame << ", " << endFrame << ")\n";

      m_encoder->prepareIntraPeriod();
      for (int frame = intraFrame; frame < endFrame; ++frame) {
        cout << "Push input frame " << (m_startFrame + frame) << '\n';
        m_encoder->pushFrame(cameras,
                             loadViews(m_startFrame + frame, cameras.size()));
      }
      m_encoder->completeIntraPeriod();

      saveMetadata(intraFrame, m_encoder->getCameras(),
                   m_encoder->getPatchList());

      for (int frame = intraFrame; frame < endFrame; ++frame) {
        cout << "Pop output atlas " << frame << '\n';
        auto atlases = m_encoder->popAtlas();
        saveViews(frame, atlases);
      }
    }
  }

private:
  CameraParameterList loadCameras() const {
    ifstream stream{json().require("SourceCameraParameters").asString()};
    if (!stream.good()) {
      throw runtime_error("Failed to load source camera parameters");
    }
    return Metadata::loadCamerasFromJson(
        Common::Json{stream}.require("cameras"),
        json().require("SourceCameraNames").asStringVector());
  }

  MVD16Frame loadViews(int inputFrame, size_t numberOfViews) const {
    MVD16Frame result(numberOfViews);

    for (size_t view = 0; view < numberOfViews; ++view) {
      auto id = json().require("SourceCameraIDs").at(view).asInt();
      {
        auto texturePath = Common::format(
            json().require("SourceTexturePathFmt").asString().c_str(), id);
        ifstream stream{texturePath};
        stream.seekg(streampos(inputFrame) * m_sourceResolution.x() *
                     m_sourceResolution.y() * 3); // YUV420P10
        result[view].first.resize(m_sourceResolution.y(),
                                  m_sourceResolution.x());
        result[view].first.read(stream);
      }
      {
        auto depthPath = Common::format(
            json().require("SourceDepthPathFmt").asString().c_str(), id);
        ifstream stream{depthPath};
        stream.seekg(streampos(inputFrame) * m_sourceResolution.x() *
                     m_sourceResolution.y() * 2); // YUV400P16
        result[view].second.resize(m_sourceResolution.y(),
                                   m_sourceResolution.x());
        result[view].second.read(stream);
      }
    }
    return result;
  }

  void saveViews(int outputFrame, const MVD16Frame &) const {}

  void saveMetadata(int outputFrame, const CameraParameterList &cameras,
                    const PatchParameterList &patches) {}

  unique_ptr<IEncoder> m_encoder;
  int m_startFrame;
  int m_numberOfFrames;
  int m_intraPeriod;
  Vec2i m_sourceResolution;
};
} // namespace TMIV::Encoder

int main(int argc, char *argv[]) {
  TMIV::Encoder::Application app{{argv, argv + argc}};
  app.run();
  return 0;
}
