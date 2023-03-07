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

#ifndef TMIV_MIVBITSTREAM_ACCESSUNIT_H
#define TMIV_MIVBITSTREAM_ACCESSUNIT_H

#include "AtlasAdaptationParameterSetRBSP.h"
#include "AtlasFrameParameterSetRBSP.h"
#include "AtlasSequenceParameterSetRBSP.h"
#include "AtlasTileLayerRBSP.h"
#include "GeometryUpscalingParameters.h"
#include "PatchParamsList.h"
#include "SequenceConfig.h"
#include "Tile.h"
#include "V3cParameterSet.h"
#include "ViewParamsList.h"
#include "ViewingSpace.h"
#include "ViewportCameraParameters.h"
#include "ViewportPosition.h"

#include <TMIV/Common/Frame.h>

namespace TMIV::MivBitstream {
struct AtlasAccessUnit {
  AtlasSequenceParameterSetRBSP asps;
  AtlasFrameParameterSetRBSP afps;
  Common::Frame<Common::PatchIdx> blockToPatchMap;
  PatchParamsList patchParamsList;
  std::vector<MivBitstream::TilePartition> tileParamsList;

  // ISO/IEC 23090-12 Annex 9
  Common::Frame<> decOccFrame;
  Common::Frame<> decGeoFrame;
  Common::FrameList<> decAttrFrame;
  Common::Frame<> decPckFrame;

  // ISO/IEC 23090-12 Annex B
  Common::Frame<> unpckOccFrame;
  Common::Frame<> unpckGeoFrame;
  Common::FrameList<> unpckAttrFrame;
  Common::Frame<bool> occFrameNF;
  Common::Frame<> geoFrameNF;
  Common::FrameList<> attrFrameNF;

  // Application of hypothetical reference render processes (Annex H)
  Common::Frame<bool> occFrame;
  Common::Frame<> geoFrame;
  Common::Frame<> texFrame;

  // Index into the block to patch map using nominal atlas coordinates
  [[nodiscard]] auto patchIdx(uint32_t row, uint32_t column) const -> uint16_t {
    const auto k = asps.asps_log2_patch_packing_block_size();
    return blockToPatchMap.getPlane(0)(row >> k, column >> k);
  }

  // A filtered, nominal resolution variant on the blockToPatchMap (not specified)
  Common::Frame<Common::PatchIdx> pixelToPatchMap;

  // Index into the pixel to patch map using nominal atlas coordinates
  [[nodiscard]] auto filteredPatchIdx(uint32_t row, uint32_t column) const -> uint16_t {
    if (pixelToPatchMap.empty()) {
      return patchIdx(row, column);
    }
    return pixelToPatchMap.getPlane(0)(row, column);
  }
};

struct AccessUnit {
  int32_t foc{-1};      // Frame order count in the coded sequence, resets to zero at each IRAP
  int32_t frameIdx{-1}; // Consecutive frame index (0-based)

  V3cParameterSet vps;
  std::optional<CommonAtlasSequenceParameterSetRBSP> casps;
  ViewParamsList viewParamsList;
  std::vector<AtlasAccessUnit> atlas;

  std::optional<ViewingSpace> vs;
  std::optional<ViewportCameraParameters> vcp;
  std::optional<ViewportPosition> vp;
  std::optional<VuiParameters> vui;
  std::optional<GeometryUpscalingParameters> gup;

  [[nodiscard]] auto sequenceConfig() const -> SequenceConfig;
};

void requireAllPatchesWithinProjectionPlaneBounds(const ViewParamsList &vpl,
                                                  const PatchParamsList &ppl);

void requireAllPatchesWithinAtlasFrameBounds(const PatchParamsList &ppl,
                                             const AtlasSequenceParameterSetRBSP &asps);
} // namespace TMIV::MivBitstream

#endif
