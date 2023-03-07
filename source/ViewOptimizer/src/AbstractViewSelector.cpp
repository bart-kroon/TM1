/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <TMIV/ViewOptimizer/AbstractViewSelector.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/LoggingStrategy.h>

namespace TMIV::ViewOptimizer {
AbstractViewSelector::AbstractViewSelector(const Common::Json & /* rootNode */,
                                           const Common::Json &componentNode)
    : m_outputAdditionalViews{componentNode.require("outputAdditionalViews").as<bool>()} {
  if (const auto &node = componentNode.optional("verticalInhomogenityCoefficient")) {
    m_verticalInhomogenityCoefficient = node.as<double>();
  } else {
    m_verticalInhomogenityCoefficient = 1.;
  }
  VERIFY(0. < m_verticalInhomogenityCoefficient && m_verticalInhomogenityCoefficient <= 1.);
}

auto AbstractViewSelector::optimizeParams(const SourceParams &params) -> ViewOptimizerParams {
  m_params = {params.viewParamsList};

  double weight = 1.0;
  if (params.viewParamsList[0].ci.ci_cam_type() == MivBitstream::CiCamType::equirectangular) {
    weight = m_verticalInhomogenityCoefficient;
  }

  m_isBasicView = isBasicView(weight);

  for (size_t i = 0; i < m_params.viewParamsList.size(); ++i) {
    m_params.viewParamsList[i].isBasicView = m_isBasicView[i];
  }

  printSummary();

  inplaceEraseAdditionalViews(m_params.viewParamsList);
  m_params.viewParamsList.constructViewIdIndex();
  return m_params;
}

auto AbstractViewSelector::optimizeFrame(Common::DeepFrameList views) const
    -> Common::DeepFrameList {
  inplaceEraseAdditionalViews(views);
  return views;
}

template <typename T>
void AbstractViewSelector::inplaceEraseAdditionalViews(std::vector<T> &views) const {
  PRECONDITION(views.size() == m_isBasicView.size());
  if (!m_outputAdditionalViews) {
    for (int32_t i = static_cast<int32_t>(views.size()) - 1; i >= 0; --i) {
      if (!m_isBasicView[i]) {
        views.erase(views.begin() + i);
      }
    }
  }
}

void AbstractViewSelector::printSummary() const {
  std::ostringstream what;

  what << "Basic views:";
  for (size_t i = 0; i < m_isBasicView.size(); ++i) {
    if (m_isBasicView[i]) {
      what << ' ' << m_params.viewParamsList[i].name;
    }
  }
  Common::logInfo(what.str());

  if (m_outputAdditionalViews) {
    what.str({});

    what << "Additional views:";
    for (size_t i = 0; i < m_isBasicView.size(); ++i) {
      if (!m_isBasicView[i]) {
        what << ' ' << m_params.viewParamsList[i].name;
      }
    }
    Common::logInfo(what.str());
  } else {
    Common::logInfo("No additional views.");
  }
}
} // namespace TMIV::ViewOptimizer
