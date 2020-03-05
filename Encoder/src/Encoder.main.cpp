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
#include <TMIV/DepthQualityAssessor/IDepthQualityAssessor.h>
#include <TMIV/IO/IO.h>
#include <TMIV/IO/IvMetadataWriter.h>
#include <TMIV/MivBitstream/MivDecoder.h>

#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::IO;
using namespace TMIV::MivBitstream;
using namespace TMIV::Decoder;
using namespace TMIV::DepthQualityAssessor;

using Mat1w = TMIV::Common::heap::Matrix<uint16_t>;

namespace TMIV::MivBitstream {
const MivDecoder::Mode MivDecoder::mode = MivDecoder::Mode::MIV;
}

namespace TMIV::Encoder {
class Application : public Common::Application {
private:
  unique_ptr<IEncoder> m_encoder;
  unique_ptr<IDepthQualityAssessor> m_depthQualityAssessor;
  int m_numberOfFrames{};
  int m_intraPeriod{};
  IvMetadataWriter m_metadataWriter;
  SizeVector m_viewSizes;

public:
  explicit Application(vector<const char *> argv)
      : Common::Application{"Encoder", move(argv)}, m_encoder{create<IEncoder>("Encoder")},
        m_depthQualityAssessor{create<IDepthQualityAssessor>("DepthQualityAssessor")},
        m_metadataWriter{json()},
        m_numberOfFrames{json().require("numberOfFrames").asInt()},
        m_intraPeriod{json().require("intraPeriod").asInt()} {}

  void run() override {
    auto sourceSequenceParams = loadSourceIvSequenceParams(json());
    m_viewSizes = sourceSequenceParams.viewParamsList.viewSizes();

    // TODO(BK): Move depth quality assessment to the Encoder library
    if (!json().isPresent("depthLowQualityFlag")) {
      sourceSequenceParams.msp().msp_depth_low_quality_flag(
          m_depthQualityAssessor->isLowDepthQuality(sourceSequenceParams,
                                                    loadSourceFrame(json(), m_viewSizes, 0)));
    }

    const auto &codedSequenceParams = m_encoder->prepareSequence(sourceSequenceParams);
    m_metadataWriter.writeIvSequenceParams(codedSequenceParams);

    for (int i = 0; i < m_numberOfFrames; i += m_intraPeriod) {
      int lastFrame = min(m_numberOfFrames, i + m_intraPeriod);
      encodeAccessUnit(i, lastFrame);
    }
  }

private:
  void encodeAccessUnit(int firstFrame, int lastFrame) {
    cout << "Access unit: [" << firstFrame << ", " << lastFrame << ")\n";
    m_encoder->prepareAccessUnit(loadSourceIvAccessUnitParams(json()));
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
