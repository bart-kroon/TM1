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

#ifndef _TMIV_MIVBITSTREAM_PATCHPARAMSLIST_H_
#define _TMIV_MIVBITSTREAM_PATCHPARAMSLIST_H_

#include <TMIV/MivBitstream/AtlasTileLayerRBSP.h>

#include <TMIV/Common/Vector.h>

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// PatchParams is the in-memory representation of PatchDataUnit (PDU). The PDU is not suitable for
// in-memory use because of the delta coding and quantization of some of the fields.
struct PatchParams {
  AtlasId atlasId{};

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
  [[nodiscard]] constexpr auto atlasPatchEntityId() const noexcept;
  [[nodiscard]] constexpr auto atlasPatchDepthOccMapThreshold() const noexcept;

  constexpr auto atlasPatch2dPosX(std::uint32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch2dPosY(std::uint32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch2dSizeX(std::uint32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch2dSizeY(std::uint32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dOffsetU(std::uint32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dOffsetV(std::uint32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dOffsetD(std::uint32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatch3dRangeD(std::uint32_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatchProjectionId(std::uint16_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatchOrientationIndex(FlexiblePatchOrientation value) noexcept
      -> PatchParams &;
  constexpr auto atlasPatchEntityId(std::uint16_t value) noexcept -> PatchParams &;
  constexpr auto atlasPatchDepthOccMapThreshold(std::uint32_t value) noexcept -> PatchParams &;

  // Is the patch rotated such that width and height swap?
  [[nodiscard]] constexpr auto isRotated() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dSizeU() const noexcept;
  [[nodiscard]] constexpr auto atlasPatch3dSizeV() const noexcept;
  constexpr auto atlasPatch3dSizeU(std::uint32_t value) noexcept;
  constexpr auto atlasPatch3dSizeV(std::uint32_t value) noexcept;

  // Pixel position conversion from atlas to/from view
  [[nodiscard]] auto viewToAtlas(Common::Vec2i viewPosition) const -> Common::Vec2i;
  [[nodiscard]] auto atlasToView(Common::Vec2i atlasPosition) const -> Common::Vec2i;

  auto operator==(const PatchParams &other) const -> bool;
  auto operator!=(const PatchParams &other) const -> bool { return !operator==(other); };

private:
  std::uint32_t m_atlasPatch2dPosX{};
  std::uint32_t m_atlasPatch2dPosY{};
  std::uint32_t m_atlasPatch2dSizeX{};
  std::uint32_t m_atlasPatch2dSizeY{};
  std::uint32_t m_atlasPatch3dOffsetU{};
  std::uint32_t m_atlasPatch3dOffsetV{};
  std::uint32_t m_atlasPatch3dOffsetD{};
  std::uint32_t m_atlasPatch3dRangeD{};
  std::uint16_t m_atlasPatchProjectionId{};
  FlexiblePatchOrientation m_atlasPatchOrientationIndex{FlexiblePatchOrientation::FPO_INVALID};
  std::optional<std::uint16_t> m_atlasPatchEntityId;
  std::optional<std::uint32_t> m_atlasPatchDepthOccMapThreshold;
};

using PatchParamsList = std::vector<PatchParams>;
} // namespace TMIV::MivBitstream

#include "PatchParamsList.hpp"

#endif
