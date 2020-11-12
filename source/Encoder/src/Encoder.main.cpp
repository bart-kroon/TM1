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

using Mat1w = TMIV::Common::heap::Matrix<uint16_t>;

namespace TMIV::Encoder {
void registerComponents();

class Application : public Common::Application {
private:
  std::unique_ptr<IEncoder> m_encoder;
  std::unique_ptr<DepthQualityAssessor::IDepthQualityAssessor> m_depthQualityAssessor;
  IvMetadataWriter m_metadataWriter;
  int m_numberOfFrames{};
  int m_intraPeriod{};
  Common::SizeVector m_viewSizes;
  std::vector<std::string> m_viewNames;

public:
  explicit Application(std::vector<const char *> argv)
      : Common::Application{"Encoder", std::move(argv)}
      , m_encoder{create<IEncoder>("Encoder")}
      , m_depthQualityAssessor{create<DepthQualityAssessor::IDepthQualityAssessor>(
            "DepthQualityAssessor")}
      , m_metadataWriter{json()}
      , m_numberOfFrames{json().require("numberOfFrames").as<int>()}
      , m_intraPeriod{json().require("intraPeriod").as<int>()} {}

  void run() override {
    auto sourceParams = IO::loadSourceParams(json());
    m_viewSizes = sourceParams.viewParamsList.viewSizes();
    m_viewNames = sourceParams.viewParamsList.viewNames();

    if (!json().optional("depthLowQualityFlag") &&
        json().optional("haveGeometryVideo").as<bool>()) {
      sourceParams.vme().vme_depth_low_quality_flag(m_depthQualityAssessor->isLowDepthQuality(
          sourceParams.viewParamsList, IO::loadSourceFrame(json(), m_viewSizes, m_viewNames, 0)));
    }
    m_encoder->prepareSequence(sourceParams);

    for (int i = 0; i < m_numberOfFrames; i += m_intraPeriod) {
      int lastFrame = std::min(m_numberOfFrames, i + m_intraPeriod);
      encodeAccessUnit(i, lastFrame);
    }

    const auto maxLumaSamplesPerFrame = m_encoder->maxLumaSamplesPerFrame();
    std::cout << "Maximum luma samples per frame is " << maxLumaSamplesPerFrame << '\n';
    m_metadataWriter.reportSummary(std::cout, m_numberOfFrames);
  }

private:
  void encodeAccessUnit(int firstFrame, int lastFrame) {
    std::cout << "Access unit: [" << firstFrame << ", " << lastFrame << ")\n";
    m_encoder->prepareAccessUnit();
    pushFrames(firstFrame, lastFrame);
    m_metadataWriter.writeAccessUnit(m_encoder->completeAccessUnit());
    popAtlases(firstFrame, lastFrame);
  }

  void pushFrames(int firstFrame, int lastFrame) {
    for (int i = firstFrame; i < lastFrame; ++i) {
      m_encoder->pushFrame(IO::loadSourceFrame(json(), m_viewSizes, m_viewNames, i));
    }
  }

  void popAtlases(int firstFrame, int lastFrame) {
    for (int i = firstFrame; i < lastFrame; ++i) {
      IO::saveAtlas(json(), i, m_encoder->popAtlas());
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
  } catch (std::runtime_error &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}