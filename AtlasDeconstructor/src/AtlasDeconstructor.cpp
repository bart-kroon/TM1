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

#include <TMIV/AtlasDeconstructor/AtlasDeconstructor.h>
#include <TMIV/Common/Factory.h>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::AtlasDeconstructor {

AtlasDeconstructor::AtlasDeconstructor(const Common::Json &) {}

PatchIdMapList
AtlasDeconstructor::getPatchIdMap(const std::vector<Vec2i> &atlasSize,
                                  const PatchParameterList &patchList) {
  PatchIdMapList patchMapList;

  for (const auto &sz : atlasSize)
  {
	  PatchIdMap patchMap(sz.x(), sz.y());
	  std::fill(patchMap.getPlane(0).begin(), patchMap.getPlane(0).end(), static_cast<uint16_t>(65535));
	  patchMapList.push_back(std::move(patchMap));
  }

  for (auto id = 0u; id < patchList.size(); id++)
    writePatchIdInMap(patchList[id], patchMapList, static_cast<uint16_t>(id));

  return patchMapList;
}

void AtlasDeconstructor::writePatchIdInMap(const PatchParameters &patch,
                                           PatchIdMapList &patchMapList,
                                           std::uint16_t patchId) {
  auto &patchMap = patchMapList[patch.atlasId];

  const Vec2i &q0 = patch.patchPackingPos;
  int w = patch.patchSize.x(), h = patch.patchSize.y();
  bool isRotated = (patch.patchRotation != Metadata::PatchRotation::upright);
  int xMin = q0.x(), xLast = q0.x() + (isRotated ? h : w);
  int yMin = q0.y(), yLast = q0.y() + (isRotated ? w : h);

  for (auto y = yMin; y < yLast; y++)
    std::fill(patchMap.getPlane(0).row_begin(y) + xMin,
              patchMap.getPlane(0).row_begin(y) + xLast, patchId);
}

} // namespace TMIV::AtlasDeconstructor
