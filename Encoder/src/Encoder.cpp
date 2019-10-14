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

#include <TMIV/Encoder/Encoder.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/Image/Image.h>

using namespace std;
using namespace TMIV::AtlasConstructor;
using namespace TMIV::Common;
using namespace TMIV::Image;
using namespace TMIV::Metadata;
using namespace TMIV::ViewOptimizer;

namespace TMIV::Encoder {
Encoder::Encoder(const Json &rootNode, const Json &componentNode) {
  m_viewOptimizer =
      Factory<IViewOptimizer>::getInstance().create("ViewOptimizer", rootNode, componentNode);
  m_atlasConstructor =
      Factory<IAtlasConstructor>::getInstance().create("AtlasConstructor", rootNode, componentNode);
}

auto Encoder::prepareSequence(Metadata::IvSequenceParams ivSequenceParams)
    -> const Metadata::IvSequenceParams & {
  auto optimized = m_viewOptimizer->optimizeSequence(move(ivSequenceParams));
  m_constructedSequenceParams =
      m_atlasConstructor->prepareSequence(move(optimized.basic), move(optimized.additional));
  m_codedSequenceParams = m_constructedSequenceParams.modifyDepthRange();
  return m_codedSequenceParams;
}

void Encoder::prepareAccessUnit(Metadata::IvAccessUnitParams ivAccessUnitParams) {
  return m_atlasConstructor->prepareAccessUnit(ivAccessUnitParams);
}

void Encoder::pushFrame(Common::MVD16Frame views) {
  auto optimized = m_viewOptimizer->optimizeFrame(move(views));
  return m_atlasConstructor->pushFrame(move(optimized.basic), move(optimized.additional));
}

auto Encoder::completeAccessUnit() -> const Metadata::IvAccessUnitParams & {
  return m_atlasConstructor->completeAccessUnit();
}

auto Encoder::popAtlas() -> Common::MVD10Frame {
  return modifyDepthRange(m_atlasConstructor->popAtlas(),
                          m_constructedSequenceParams.viewParamsList,
                          m_codedSequenceParams.viewParamsList);
}
} // namespace TMIV::Encoder
