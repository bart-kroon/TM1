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

#include <TMIV/Encoder/Encoder.h>

#include <cassert>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Encoder {
auto Encoder::prepareSequence(IvSequenceParams sourceIvs) -> const IvSequenceParams & {
  // Transform source to transport view sequence parameters
  tie(m_transportIvs, m_isBasicView) = m_viewOptimizer->optimizeSequence(move(sourceIvs));

  // Construct at least the basic views
  if (m_transportIvs.msp().msp_max_entities_minus1() == 0) {
    m_nbAtlas = max(static_cast<size_t>(count(m_isBasicView.begin(), m_isBasicView.end(), true)),
                    m_nbAtlas);
  }

  // Create IVS with VPS with right number of atlases but copy other parts from input IVS
  m_ivs = IvSequenceParams{SizeVector(m_nbAtlas, m_atlasSize), haveTexture()};
  m_ivs.msp() = m_transportIvs.msp();
  m_ivs.viewParamsList = m_transportIvs.viewParamsList;
  m_ivs.viewingSpace = m_transportIvs.viewingSpace;

  // Register pruning relation
  m_pruner->registerPruningRelation(m_ivs, m_isBasicView);

  // Turn on occupancy coding per view
  enableOccupancyPerView();

  // Further transform sequence parameters: geometry downscaling and depth/occupancy coding
  return m_geometryDownscaler.transformSequenceParams(
      m_depthOccupancy->transformSequenceParams(m_ivs));
}

auto Encoder::haveTexture() const -> bool {
  assert(m_transportIvs.vps.vps_atlas_count_minus1() == 0);
  const auto &ai = m_transportIvs.vps.attribute_information(0);
  return ai.ai_attribute_count() >= 1 &&
         ai.ai_attribute_type_id(0) == AiAttributeTypeId::ATTR_TEXTURE;
}

void Encoder::enableOccupancyPerView() {
  for (size_t viewId = 0; viewId < m_ivs.viewParamsList.size(); ++viewId) {
    if (!m_isBasicView[viewId] || m_ivs.msp().msp_max_entities_minus1() > 0) {
      m_ivs.viewParamsList[viewId].hasOccupancy = true;
    }
  }
}
} // namespace TMIV::Encoder
