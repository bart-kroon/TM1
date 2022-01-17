/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#ifndef TMIV_MIVBITSTREAM_PATCHPARAMSLIST_H
#define TMIV_MIVBITSTREAM_PATCHPARAMSLIST_H

#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/LinAlg.h>
#include <TMIV/Common/Matrix.h>
#include <TMIV/Common/Vector.h>

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// PatchParams is the in-memory representation of PatchDataUnit (PDU). The PDU is not suitable for
// in-memory use because of the delta coding and quantization of some of the fields.
class PatchParams {
public:
  [[nodiscard]] constexpr auto atlasId() const noexcept;
  constexpr auto atlasId(AtlasId value) noexcept -> decltype(auto);

  [[nodiscard]] constexpr auto atlasPatch2dPosX() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch2dPosY() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch2dSizeX() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch2dSizeY() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dOffsetU() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dOffsetV() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dOffsetD() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dRangeD() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchProjectionId() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchOrientationIndex() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchLoDScaleX() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchLoDScaleY() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchEntityId() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchDepthOccThreshold() const noexcept;
  [[nodiscard]] constexpr auto asme_depth_occ_threshold_flag() const noexcept;
  [[nodiscard]] auto atlasPatchTextureOffset() const;
  [[nodiscard]] constexpr auto atlasPatchInpaintFlag() const noexcept;

  constexpr auto atlasPatch2dPosX(int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch2dPosY(int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch2dSizeX(int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch2dSizeY(int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dOffsetU(int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dOffsetV(int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dOffsetD(Common::SampleValue value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dRangeD(Common::SampleValue value) noexcept -> PatchParams &;
  constexpr auto atlasPatchProjectionId(ViewId value) noexcept -> PatchParams &;
  constexpr auto atlasPatchOrientationIndex(FlexiblePatchOrientation value) noexcept
      -> PatchParams &;
  constexpr auto atlasPatchLoDScaleX(int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatchLoDScaleY(int32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatchEntityId(Common::SampleValue value) noexcept -> PatchParams &;
  constexpr auto atlasPatchDepthOccThreshold(uint32_t value) noexcept -> PatchParams &;
  auto atlasPatchTextureOffset(Common::Vec3w value) noexcept -> PatchParams &;
  constexpr auto atlasPatchInpaintFlag(bool value) noexcept -> PatchParams &;

  // Is the patch rotated such that width and height swap?
  [[nodiscard]] constexpr auto isRotated() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dSizeU() const noexcept -> decltype(auto);
  [[nodiscard]] constexpr auto atlasPatch3dSizeV() const noexcept -> decltype(auto);
  constexpr auto atlasPatch3dSizeU(int32_t value) noexcept -> decltype(auto);
  constexpr auto atlasPatch3dSizeV(int32_t value) noexcept -> decltype(auto);

  // Pixel position conversion from atlas (x, y) to view (u, v)
  //
  // * Implements ISO/IEC 23090-5:2021(2E) V3E [WG 07 N 0003], Eq. (49)
  // * Although compilers may be smart enough, computing the transform first may be faster
  [[nodiscard]] auto atlasToView(Common::Vec2i xy) const noexcept -> Common::Vec2i;
  [[nodiscard]] auto atlasToViewTransform() const noexcept -> Common::Mat3x3i;
  [[nodiscard]] static auto atlasToView(Common::Vec2i xy, const Common::Mat3x3i &m) noexcept
      -> Common::Vec2i;

  // Pixel position conversion from view (u, v) to atlas (x, y)
  //
  //  * Forms mapping with atlasToView when lodX == 1 and lodY == 1
  //  * For lodX > 1 or lodY > 1 only the transform is available
  //  * Although compilers may be smart enough, computing the transform first may be faster
  [[nodiscard]] auto viewToAtlas(Common::Vec2i uv) const noexcept -> Common::Vec2i;
  [[nodiscard]] auto viewToAtlasTransform() const noexcept -> Common::Mat3x3i;
  [[nodiscard]] static auto viewToAtlas(Common::Vec2i uv, const Common::Mat3x3i &m) noexcept
      -> Common::Vec2i;

  static auto decodePdu(const PatchDataUnit &pdu, const AtlasSequenceParameterSetRBSP &asps,
                        const AtlasFrameParameterSetRBSP &afps, const AtlasTileHeader &ath)
      -> PatchParams;
  [[nodiscard]] auto encodePdu(const AtlasSequenceParameterSetRBSP &asps,
                               const AtlasFrameParameterSetRBSP &afps,
                               const AtlasTileHeader &ath) const -> PatchDataUnit;

  auto operator==(const PatchParams &other) const -> bool;
  auto operator!=(const PatchParams &other) const -> bool { return !operator==(other); };

private:
  AtlasId m_atlasId{};

  int32_t m_atlasPatch2dPosX{};
  int32_t m_atlasPatch2dPosY{};
  int32_t m_atlasPatch2dSizeX{};
  int32_t m_atlasPatch2dSizeY{};
  int32_t m_atlasPatch3dOffsetU{};
  int32_t m_atlasPatch3dOffsetV{};
  Common::SampleValue m_atlasPatch3dOffsetD{};
  Common::SampleValue m_atlasPatch3dRangeD{};
  ViewId m_atlasPatchProjectionId{};
  int32_t m_atlasPatchLoDScaleX{1};
  int32_t m_atlasPatchLoDScaleY{1};
  FlexiblePatchOrientation m_atlasPatchOrientationIndex{FlexiblePatchOrientation::FPO_INVALID};
  Common::SampleValue m_atlasPatchEntityId{};
  std::optional<uint32_t> m_atlasPatchDepthOccThreshold;
  Common::Vec3w m_atlasPatchTextureOffset{};
  bool m_atlasPatchInpaintFlag{};
};

using PatchParamsList = std::vector<PatchParams>;
} // namespace TMIV::MivBitstream

#include "PatchParamsList.hpp"

#endif
