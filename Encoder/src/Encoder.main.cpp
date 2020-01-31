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

#include <TMIV/Encoder/IEncoder.h>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/Decoder/IDecoder.h>
#include <TMIV/IO/IO.h>
#include <TMIV/IO/IvMetadataWriter.h>

#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::IO;
using namespace TMIV::Metadata;
using namespace TMIV::Decoder;

using Mat1w = TMIV::Common::heap::Matrix<uint16_t>;

namespace TMIV::Encoder {
class Application : public Common::Application {
private:
  unique_ptr<IEncoder> m_encoder;
  int m_numberOfFrames{};
  int m_intraPeriod{};
  bool m_downscale_depth = false;
  IvMetadataWriter m_metadataWriter;
  SizeVector m_viewSizes;
  unique_ptr<IDecoder> m_reconstructor;

public:
  explicit Application(vector<const char *> argv)
      : Common::Application{"Encoder", move(argv)}, m_encoder{create<IEncoder>("Encoder")},
        m_metadataWriter{json(), "OutputDirectory", "AtlasMetadataPath"} {
    m_numberOfFrames = json().require("numberOfFrames").asInt();
    m_intraPeriod = json().require("intraPeriod").asInt();

    if (auto node = json().optional("reconstruct"); node && node.asBool()) {
      m_reconstructor = create<IDecoder>("Decoder");
    }
    if (auto node = json().optional("depthDownScaleFlag"); node ) {
      m_downscale_depth = node.asBool() ;
    }

  }

  void run() override {
    const auto sourceSequenceParams = loadSourceIvSequenceParams(json());
    m_viewSizes = sourceSequenceParams.viewParamsList.viewSizes();
    cout << "\nSource sequence parameters:\n" << sourceSequenceParams;

    const auto codedSequenceParams = m_encoder->prepareSequence(sourceSequenceParams);
    m_metadataWriter.writeIvSequenceParams(codedSequenceParams);
    cout << "\nCoded sequence parameters:\n" << codedSequenceParams;

    if (m_reconstructor) {
      m_reconstructor->updateSequenceParams(codedSequenceParams);
    }

    for (int i = 0; i < m_numberOfFrames; i += m_intraPeriod) {
      int lastFrame = min(m_numberOfFrames, i + m_intraPeriod);
      encodeAccessUnit(i, lastFrame);
    }
  }

private:
  void encodeAccessUnit(int firstFrame, int lastFrame) {
    cout << "Access unit: [" << firstFrame << ", " << lastFrame << ")\n";

    const auto sourceAccessUnitParams = loadSourceIvAccessUnitParams(json());
    m_encoder->prepareAccessUnit(sourceAccessUnitParams);
    cout << "\nSource access unit parameters:\n" << sourceAccessUnitParams;

    pushFrames(firstFrame, lastFrame);
    const auto codedAccessUnitParams = m_encoder->completeAccessUnit();
    cout << "\nCoded access unit parameters:\n" << codedAccessUnitParams;

    m_metadataWriter.writeIvAccessUnitParams(codedAccessUnitParams);

    if (m_reconstructor) {
      m_reconstructor->updateAccessUnitParams(codedAccessUnitParams);
    }

    popAtlases(firstFrame, lastFrame);
  }

  void pushFrames(int firstFrame, int lastFrame) {
    for (int i = firstFrame; i < lastFrame; ++i) {
      m_encoder->pushFrame(loadSourceFrame(json(), m_viewSizes, i), i - firstFrame);
    }
  }

  void popAtlases(int firstFrame, int lastFrame) {
    for (int i = firstFrame; i < lastFrame; ++i) {
      auto atlas = m_encoder->popAtlas();
      
      if (m_downscale_depth) {
        atlas = DownScaleAtlasDepth(atlas);
      }

      saveAtlas(json(), i, atlas);

      if (m_reconstructor) {
        const auto viewportParams = loadViewportMetadata(json(), i);
        const auto viewport = m_reconstructor->decodeFrame(atlas, viewportParams);
        cout << "Reconstruction: " << viewportParams << '\n';

        saveViewport(json(), i, {yuv420p(viewport.first), viewport.second});
      }
    }
  }

  static auto MaxPool2x2(const Mat1w& depth) -> Mat1w
  {
    auto w = depth.width() / 2U;
    auto h = depth.height() / 2U;
    
    Mat1w depthD2;
    depthD2.resize(h,w);
    
    for (auto i = 0U; i < h; ++i) {
      for (auto j = 0U; j < w; ++j) {
        auto d0 = depth(2 * i, 2 * j);
        auto d1 = depth(2 * i, 2 * j + 1);
        auto d2 = depth(2 * i + 1, 2 * j);
        auto d3 = depth(2 * i + 1, 2 * j + 1);

        // assume high levels indicate foreground
        depthD2(i, j) = std::max(d0, std::max(d1, std::max(d2, d3)));
      }
    }

    return depthD2;
  }

  static auto DownScaleAtlasDepth(const MVD10Frame &atlas) -> MVD10Frame
  {
    TMIV::Common::MVD10Frame atlasOut(atlas.size());

    for (auto i = 0U; i < atlas.size(); ++i) {
      // copy texture
      atlasOut[i].first = atlas[i].first;
      
      // downscale depth using max-of-4 operation
      auto depthD2 = MaxPool2x2(atlas[i].second.getPlane(0));
      atlasOut[i].second.resize( int(depthD2.width()), int(depthD2.height()) );
      atlasOut[i].second.getPlane(0) = depthD2;
    }

    return atlasOut;
  }

};
} // namespace TMIV::Encoder

#include "../../Decoder/src/Decoder.reg.hpp"
#include "Encoder.reg.hpp"

auto main(int argc, char *argv[]) -> int {
  try {
    TMIV::Encoder::registerComponents();
    TMIV::Decoder::registerComponents();
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
