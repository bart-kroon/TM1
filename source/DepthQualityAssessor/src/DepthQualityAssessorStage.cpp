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

#include <TMIV/DepthQualityAssessor/DepthQualityAssessorStage.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/Common/LoggingStrategy.h>

namespace TMIV::DepthQualityAssessor {
DepthQualityAssessorStage::DepthQualityAssessorStage(const Common::Json &rootNode,
                                                     const Common::Json &componentNode) {
  if (const auto &node = componentNode.optional("depthLowQualityFlag")) {
    m_depthLowQualityFlag = node.as<bool>();
  } else if (rootNode.require("haveGeometryVideo").as<bool>()) {
    m_assessor =
        Common::create<IDepthQualityAssessor>("DepthQualityAssessor", rootNode, componentNode);
  } else {
    m_depthLowQualityFlag = false;
  }
}

void DepthQualityAssessorStage::encode(SourceUnit unit) {
  Common::logDebug("Depth quality assessor stage");

  if (!m_depthLowQualityFlag) {
    m_depthLowQualityFlag = m_assessor->isLowDepthQuality(unit.viewParamsList, unit.deepFrameList);
  }

  unit.depthLowQualityFlag = *m_depthLowQualityFlag;
  source.encode(std::move(unit));
}
} // namespace TMIV::DepthQualityAssessor
