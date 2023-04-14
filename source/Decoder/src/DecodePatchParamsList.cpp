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

#include <TMIV/Decoder/DecodePatchParamsList.h>

namespace TMIV::Decoder {
void decodePatchParamsList(const MivBitstream::V3cParameterSet &vps,
                           MivBitstream::V3cUnitHeader vuh, const AtlasAccessUnit &au,
                           MivBitstream::TilePartition &tile, size_t tileIdx) {
  const auto &ath = au.atlV[tileIdx].atlas_tile_header();
  LIMITATION(tileIdx == ath.ath_id());

  VERIFY_MIVBITSTREAM(ath.ath_type() == MivBitstream::AthType::I_TILE ||
                      ath.ath_type() == MivBitstream::AthType::SKIP_TILE);

  if (ath.ath_type() == MivBitstream::AthType::I_TILE) {
    const auto &atdu = au.atlV[tileIdx].atlas_tile_data_unit();

    auto ppl = MivBitstream::PatchParamsList{};
    ppl.reserve(atdu.atduTotalNumberOfPatches());

    for (size_t p = 0; p < atdu.atduTotalNumberOfPatches(); ++p) {
      VERIFY_MIVBITSTREAM(atdu.atdu_patch_mode(p) == MivBitstream::AtduPatchMode::I_INTRA);
      const auto &pdu = atdu.patch_information_data(p).patch_data_unit();
      ppl.push_back(MivBitstream::PatchParams::decodePdu(pdu, vps, vuh.vuh_atlas_id(), au.asps,
                                                         au.afps, ath));
    }
    tile.partitionPatchList(ppl);
  }
}

// NOTE(BK): Combined implementation of two processes because there is only a single tile in MIV
// main profile:
//  * [WG 07 N 0003:9.2.6]   Decoding process of the block to patch map
//  * [WG 07 N 0003:9.2.7.2] Conversion of tile level blockToPatch information to atlas level
//                           blockToPatch information
auto decodeBlockToPatchMap(const MivBitstream::AtlasSequenceParameterSetRBSP &asps,
                           const MivBitstream::PatchParamsList &ppl)
    -> Common::Frame<Common::PatchIdx> {
  const int32_t log2PatchPackingBlockSize = asps.asps_log2_patch_packing_block_size();
  const auto patchPackingBlockSize = 1 << log2PatchPackingBlockSize;
  const auto offset = patchPackingBlockSize - 1;

  const auto atlasBlockToPatchMapWidth = (asps.asps_frame_width() + offset) / patchPackingBlockSize;
  const auto atlasBlockToPatchMapHeight =
      (asps.asps_frame_height() + offset) / patchPackingBlockSize;

  // All elements of TileBlockToPatchMap are key initialized to -1 as follows [9.2.6]
  auto btpm = Common::Frame<Common::PatchIdx>::lumaOnly(
      {atlasBlockToPatchMapWidth, atlasBlockToPatchMapHeight});
  btpm.fillValue(Common::unusedPatchIdx);

  // Then the AtlasBlockToPatchMap array is updated as follows:
  for (size_t p = 0; p < ppl.size(); ++p) {
    const size_t xOrg = ppl[p].atlasPatch2dPosX() / patchPackingBlockSize;
    const size_t yOrg = ppl[p].atlasPatch2dPosY() / patchPackingBlockSize;
    const size_t atlasPatchWidthBlk = (ppl[p].atlasPatch2dSizeX() + offset) / patchPackingBlockSize;
    const size_t atlasPatchHeightBlk =
        (ppl[p].atlasPatch2dSizeY() + offset) / patchPackingBlockSize;

    for (size_t y = 0; y < atlasPatchHeightBlk; ++y) {
      for (size_t x = 0; x < atlasPatchWidthBlk; ++x) {
        if (!asps.asps_patch_precedence_order_flag() ||
            btpm.getPlane(0)(yOrg + y, xOrg + x) == Common::unusedPatchIdx) {
          btpm.getPlane(0)(yOrg + y, xOrg + x) = static_cast<uint16_t>(p);
        }
      }
    }
  }
  return btpm;
}
} // namespace TMIV::Decoder
