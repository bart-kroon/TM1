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

#include <TMIV/Encoder/IEncoder.h>

#include <TMIV/Common/Application.h>
#include <TMIV/Common/Factory.h>
#include <TMIV/DepthQualityAssessor/IDepthQualityAssessor.h>
#include <TMIV/IO/IO.h>

#include "IvMetadataWriter.h"

#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::IO;
using namespace TMIV::MivBitstream;
using namespace TMIV::DepthQualityAssessor;

using Mat1w = TMIV::Common::heap::Matrix<uint16_t>;

namespace TMIV::Encoder {
void registerComponents();

class Application : public Common::Application {
private:
  unique_ptr<IEncoder> m_encoder;
  unique_ptr<IDepthQualityAssessor> m_depthQualityAssessor;
  IvMetadataWriter m_metadataWriter;
  int m_numberOfFrames{};
  int m_intraPeriod{};
  SizeVector m_viewSizes;

public:
  explicit Application(vector<const char *> argv)
      : Common::Application{"Encoder", move(argv)}
      , m_encoder{create<IEncoder>("Encoder")}
      , m_depthQualityAssessor{create<IDepthQualityAssessor>("DepthQualityAssessor")}
      , m_metadataWriter{json()}
      , m_numberOfFrames{json().require("numberOfFrames").asInt()}
      , m_intraPeriod{json().require("intraPeriod").asInt()} {}

  void run() override {
    auto sourceSequenceParams = loadSourceIvSequenceParams(json());
    m_viewSizes = sourceSequenceParams.viewParamsList.viewSizes();

    if (!json().isPresent("depthLowQualityFlag")) {
      sourceSequenceParams.vme().vme_depth_low_quality_flag(
          m_depthQualityAssessor->isLowDepthQuality(sourceSequenceParams,
                                                    loadSourceFrame(json(), m_viewSizes, 0)));
    }

    const auto &codedSequenceParams = m_encoder->prepareSequence(sourceSequenceParams);
    m_metadataWriter.writeIvSequenceParams(codedSequenceParams);

    for (int i = 0; i < m_numberOfFrames; i += m_intraPeriod) {
      int lastFrame = min(m_numberOfFrames, i + m_intraPeriod);
      encodeAccessUnit(i, lastFrame);
    }

    const auto maxLumaSamplesPerFrame = m_encoder->maxLumaSamplesPerFrame();
    cout << "Maximum luma samples per frame is " << maxLumaSamplesPerFrame << '\n';
    m_metadataWriter.reportSummary(cout);
  }

private:
  void encodeAccessUnit(int firstFrame, int lastFrame) {
    cout << "Access unit: [" << firstFrame << ", " << lastFrame << ")\n";
    m_encoder->prepareAccessUnit({});
    pushFrames(firstFrame, lastFrame);
    m_metadataWriter.writeIvAccessUnitParams(m_encoder->completeAccessUnit(),
                                             lastFrame - firstFrame);
    popAtlases(firstFrame, lastFrame);
  }

  void pushFrames(int firstFrame, int lastFrame) {
    for (int i = firstFrame; i < lastFrame; ++i) {
      m_encoder->pushFrame(loadSourceFrame(json(), m_viewSizes, i));
    }
  }

  void popAtlases(int firstFrame, int lastFrame) {
    for (int i = firstFrame; i < lastFrame; ++i) {
      saveAtlas(json(), i, m_encoder->popAtlas());
    }
  }
};
} // namespace TMIV::Encoder

auto main(int argc, char *argv[]) -> int {
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
