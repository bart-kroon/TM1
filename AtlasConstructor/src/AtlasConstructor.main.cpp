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

#include <TMIV/AtlasConstructor/Aggregator.h>
#include <TMIV/AtlasConstructor/AtlasConstructor.h>
#include <TMIV/AtlasConstructor/Packer.h>
#include <TMIV/AtlasConstructor/Pruner.h>
#include <TMIV/Common/Application.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Renderer/Synthesizer.h>
#include <fstream>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::Metadata;
using namespace TMIV::Renderer;

namespace TMIV::AtlasConstructor {

class Application : public Common::Application {
private:
  class ComponentRegistrator {
  public:
    ComponentRegistrator() {
      Factory<IAtlasConstructor>::getInstance().registerAs<AtlasConstructor>(
          "AtlasConstructor");
      Factory<ISynthesizer>::getInstance().registerAs<Synthesizer>(
          "Synthesizer");
      Factory<IPruner>::getInstance().registerAs<Pruner>("Pruner");
      Factory<IAggregator>::getInstance().registerAs<Aggregator>("Aggregator");
      Factory<IPacker>::getInstance().registerAs<Packer>("Packer");
    }
  };

private:
  std::string m_sourceDirectory = ".";
  std::string m_outputDirectory = ".";
  int m_startFrame = 0;
  int m_numberOfFrames = 32;
  int m_intraPeriod = 32;
  std::vector<int> m_baseViewId = {0};
  std::vector<int> m_additionalViewId = {1};
  unique_ptr<IAtlasConstructor> m_atlasContructor;

public:
  Application(vector<const char *> argv)
      : Common::Application{"AtlasConstructor", move(argv)} {
    static ComponentRegistrator componentRegistrator;

    if (auto subnode = json().optional("SourceDirectory"))
      m_sourceDirectory = subnode.asString();

    if (auto subnode = json().optional("OutputDirectory"))
      m_outputDirectory = subnode.asString();

    m_startFrame = json().require("startFrame").asInt();
    m_numberOfFrames = json().require("numberOfFrames").asInt();
    m_intraPeriod = json().require("intraPeriod").asInt();

    if (auto subnode = json().optional("BaseView")) {
      m_baseViewId.clear();

      for (auto i = 0u; i < subnode.size(); i++)
        m_baseViewId.push_back(subnode.at(i).asInt());
    }

    if (auto subnode = json().optional("AdditionalView")) {
      m_additionalViewId.clear();

      for (auto i = 0u; i < subnode.size(); i++)
        m_additionalViewId.push_back(subnode.at(i).asInt());
    }

    m_atlasContructor = create<IAtlasConstructor>("AtlasConstructor");
  }

  void run() override {
    // Loading cameras
    auto allCameras = loadCameras();

    for (int intraFrame = 0; intraFrame < m_numberOfFrames;
         intraFrame += m_intraPeriod) {
      int endFrame = min(m_numberOfFrames, intraFrame + m_intraPeriod);
      cout << "Intra period: [" << intraFrame << ", " << endFrame << ")\n";

      m_atlasContructor->prepareIntraPeriod();

      for (int frame = intraFrame; frame < endFrame; ++frame) {
        MVDFrame allViews =
            loadViews(allCameras, m_startFrame + frame, allCameras.size());

        // Lazy view optimization
        MVDFrame baseViews, additionalViews;
        CameraParameterList baseCameras, additionalCameras;

        for (auto id : m_baseViewId) {
          baseViews.push_back(std::move(allViews[id]));
          baseCameras.push_back(allCameras[id]);
        }

        for (auto id : m_additionalViewId) {
          additionalViews.push_back(std::move(allViews[id]));
          additionalCameras.push_back(allCameras[id]);
        }

        // Push frame
        cout << "Push input frame " << (m_startFrame + frame) << '\n';
        m_atlasContructor->pushFrame(baseCameras, baseViews, additionalCameras,
                                     additionalViews);
      }

      m_atlasContructor->completeIntraPeriod();

      saveMetadata(intraFrame, m_atlasContructor->getCameras(),
                   m_atlasContructor->getPatchList());

      for (int frame = intraFrame; frame < endFrame; ++frame) {
        cout << "Pop output atlas " << frame << '\n';
        auto atlas = m_atlasContructor->popAtlas();
        saveViews(frame, atlas);
      }
    }
  }

private:
  Metadata::CameraParameterList loadCameras() const {
    ifstream stream{m_sourceDirectory + "/" +
                    json().require("SourceCameraParameters").asString()};

    if (!stream.good()) {
      throw runtime_error("Failed to load source camera parameters");
    }

    return Metadata::loadCamerasFromJson(
        Common::Json{stream}.require("cameras"),
        json().require("SourceCameraNames").asStringVector());
  }
  MVDFrame loadViews(const Metadata::CameraParameterList &cameras,
                     int inputFrame, size_t numberOfViews) const {
    MVDFrame result(numberOfViews);

    for (auto view = 0u; view < numberOfViews; ++view) {
      Vec2i sourceResolution = cameras[view].size;
      auto id = json().require("SourceCameraNames").at(view).asString();

      {
        auto texturePath = Common::format(
            json().require("SourceTexturePathFmt").asString().c_str(),
            id.c_str());
        ifstream stream(m_sourceDirectory + "/" + texturePath,
                        ifstream::binary);

        stream.seekg(streampos(inputFrame) * sourceResolution.x() *
                     sourceResolution.y() * 3); // YUV420P10
        result[view].first.resize(sourceResolution.x(), sourceResolution.y());
        result[view].first.read(stream);
      }

      {
        auto depthPath = Common::format(
            json().require("SourceDepthPathFmt").asString().c_str(),
            id.c_str());
        ifstream stream(m_sourceDirectory + "/" + depthPath, ifstream::binary);

        stream.seekg(streampos(inputFrame) * sourceResolution.x() *
                     sourceResolution.y() * 2); // YUV400P16
        result[view].second.resize(sourceResolution.x(), sourceResolution.y());
        result[view].second.read(stream);
      }
    }

    return result;
  }
  void saveViews(int outputFrame, const MVDFrame &atlas) const {
    std::vector<Frame<YUV420P10>> outputDepth;

    for (unsigned id = 0; id < atlas.size(); id++) {
      {
        auto texturePath = Common::format(
            json().require("AtlasTexturePathFmt").asString().c_str(), id);
        ofstream os(m_outputDirectory + "/" + texturePath,
                    ((outputFrame == 0) ? ofstream::trunc : ofstream::app) |
                        ofstream::binary);
        atlas[id].first.dump(os);
      }

      {
        auto depthPath = Common::format(
            json().require("AtlasDepthPathFmt").asString().c_str(), id);
        ofstream os(m_outputDirectory + "/" + depthPath,
                    ((outputFrame == 0) ? ofstream::trunc : ofstream::app) |
                        ofstream::binary);
        Frame<YUV420P10> outputDepthFrame(atlas[id].second.getWidth(),
                                          atlas[id].second.getHeight());

        convert(atlas[id].second, outputDepthFrame);

        outputDepthFrame.dump(os);
      }
    }
  }
  void saveMetadata(int outputFrame, const CameraParameterList &cameras,
                    const PatchParameterList &patches) {
    // TODO
  }
};

} // namespace TMIV::AtlasConstructor

int main(int argc, char *argv[]) {
  TMIV::AtlasConstructor::Application app{{argv, argv + argc}};
  app.run();
  return 0;
}
