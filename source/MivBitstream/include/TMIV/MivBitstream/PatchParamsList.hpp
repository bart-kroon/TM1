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
#error "Include the .h, not the .hpp"
#endif

#include <cassert>

namespace TMIV::MivBitstream {
constexpr auto PatchParams::atlasId() const noexcept { return m_atlasId; }

constexpr auto PatchParams::atlasId(AtlasId value) noexcept -> decltype(auto) {
  m_atlasId = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch2dPosX() const noexcept { return m_atlasPatch2dPosX; }

constexpr auto PatchParams::atlasPatch2dPosY() const noexcept { return m_atlasPatch2dPosY; }

constexpr auto PatchParams::atlasPatch2dSizeX() const noexcept { return m_atlasPatch2dSizeX; }

constexpr auto PatchParams::atlasPatch2dSizeY() const noexcept { return m_atlasPatch2dSizeY; }

constexpr auto PatchParams::atlasPatch3dOffsetU() const noexcept { return m_atlasPatch3dOffsetU; }

constexpr auto PatchParams::atlasPatch3dOffsetV() const noexcept { return m_atlasPatch3dOffsetV; }

constexpr auto PatchParams::atlasPatch3dOffsetD() const noexcept { return m_atlasPatch3dOffsetD; }

constexpr auto PatchParams::atlasPatch3dRangeD() const noexcept { return m_atlasPatch3dRangeD; }

constexpr auto PatchParams::atlasPatchProjectionId() const noexcept {
  return m_atlasPatchProjectionId;
}

constexpr auto PatchParams::atlasPatchOrientationIndex() const noexcept {
  return m_atlasPatchOrientationIndex;
}

constexpr auto PatchParams::atlasPatchLoDScaleX() const noexcept { return m_atlasPatchLoDScaleX; }

constexpr auto PatchParams::atlasPatchLoDScaleY() const noexcept { return m_atlasPatchLoDScaleY; }

constexpr auto PatchParams::atlasPatchEntityId() const noexcept { return m_atlasPatchEntityId; }

constexpr auto PatchParams::atlasPatchDepthOccThreshold() const noexcept {
  return m_atlasPatchDepthOccThreshold.value_or(0);
}

constexpr auto PatchParams::asme_depth_occ_threshold_flag() const noexcept {
  return m_atlasPatchDepthOccThreshold.has_value();
}

inline auto PatchParams::atlasPatchTextureOffset(uint8_t c) const {
  return m_atlasPatchTextureOffset[c];
}

constexpr auto PatchParams::atlasPatchInpaintFlag() const noexcept {
  return m_atlasPatchInpaintFlag;
}

constexpr auto PatchParams::atlasPatch2dPosX(int32_t value) noexcept -> PatchParams & {
  m_atlasPatch2dPosX = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch2dPosY(int32_t value) noexcept -> PatchParams & {
  m_atlasPatch2dPosY = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch2dSizeX(int32_t value) noexcept -> PatchParams & {
  m_atlasPatch2dSizeX = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch2dSizeY(int32_t value) noexcept -> PatchParams & {
  m_atlasPatch2dSizeY = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch3dOffsetU(int32_t value) noexcept -> PatchParams & {
  m_atlasPatch3dOffsetU = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch3dOffsetV(int32_t value) noexcept -> PatchParams & {
  m_atlasPatch3dOffsetV = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch3dOffsetD(Common::SampleValue value) noexcept
    -> PatchParams & {
  m_atlasPatch3dOffsetD = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch3dRangeD(Common::SampleValue value) noexcept
    -> PatchParams & {
  m_atlasPatch3dRangeD = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchProjectionId(ViewId value) noexcept -> PatchParams & {
  m_atlasPatchProjectionId = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchOrientationIndex(FlexiblePatchOrientation value) noexcept
    -> PatchParams & {
  m_atlasPatchOrientationIndex = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchLoDScaleX(int32_t value) noexcept -> PatchParams & {
  m_atlasPatchLoDScaleX = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchLoDScaleY(int32_t value) noexcept -> PatchParams & {
  m_atlasPatchLoDScaleY = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchEntityId(Common::SampleValue value) noexcept
    -> PatchParams & {
  m_atlasPatchEntityId = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchDepthOccThreshold(uint32_t value) noexcept -> PatchParams & {
  m_atlasPatchDepthOccThreshold = value;
  return *this;
}

inline auto PatchParams::atlasPatchTextureOffset(uint8_t c, int32_t value) noexcept
    -> PatchParams & {
  m_atlasPatchTextureOffset[c] = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchInpaintFlag(bool value) noexcept -> PatchParams & {
  m_atlasPatchInpaintFlag = value;
  return *this;
}

inline auto PatchParams::operator==(const PatchParams &other) const -> bool {
  return atlasId() == other.atlasId() && atlasPatch2dPosX() == other.atlasPatch2dPosX() &&
         atlasPatch2dPosY() == other.atlasPatch2dPosY() &&
         atlasPatch2dSizeX() == other.atlasPatch2dSizeX() &&
         atlasPatch2dSizeY() == other.atlasPatch2dSizeY() &&
         atlasPatch3dOffsetU() == other.atlasPatch3dOffsetU() &&
         atlasPatch3dOffsetV() == other.atlasPatch3dOffsetV() &&
         atlasPatch3dOffsetD() == other.atlasPatch3dOffsetD() &&
         atlasPatch3dRangeD() == other.atlasPatch3dRangeD() &&
         atlasPatchProjectionId() == other.atlasPatchProjectionId() &&
         atlasPatchOrientationIndex() == other.atlasPatchOrientationIndex() &&
         atlasPatchLoDScaleX() == other.atlasPatchLoDScaleX() &&
         atlasPatchLoDScaleY() == other.atlasPatchLoDScaleY() &&
         atlasPatchEntityId() == other.atlasPatchEntityId() &&
         atlasPatchDepthOccThreshold() == other.atlasPatchDepthOccThreshold() &&
         m_atlasPatchTextureOffset == other.m_atlasPatchTextureOffset &&
         atlasPatchInpaintFlag() == other.atlasPatchInpaintFlag();
}

constexpr auto PatchParams::isRotated() const noexcept {
  ASSERT(atlasPatchOrientationIndex() <= FlexiblePatchOrientation::FPO_MROT180);

  return atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_ROT90 ||
         atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_SWAP ||
         atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_ROT270 ||
         atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_MROT90;
}

constexpr auto PatchParams::atlasPatch3dSizeU() const noexcept -> decltype(auto) {
  return isRotated() ? atlasPatch2dSizeY() : atlasPatch2dSizeX();
}

constexpr auto PatchParams::atlasPatch3dSizeV() const noexcept -> decltype(auto) {
  return isRotated() ? atlasPatch2dSizeX() : atlasPatch2dSizeY();
}

constexpr auto PatchParams::atlasPatch3dSizeU(int32_t value) noexcept -> decltype(auto) {
  return isRotated() ? atlasPatch2dSizeY(value) : atlasPatch2dSizeX(value);
}

constexpr auto PatchParams::atlasPatch3dSizeV(int32_t value) noexcept -> decltype(auto) {
  return isRotated() ? atlasPatch2dSizeX(value) : atlasPatch2dSizeY(value);
}

inline auto PatchParams::atlasToView(Common::Vec2i xy) const noexcept -> Common::Vec2i {
  return atlasToView(xy, atlasToViewTransform());
}

inline auto PatchParams::viewToAtlas(Common::Vec2i uv) const noexcept -> Common::Vec2i {
  return viewToAtlas(uv, viewToAtlasTransform());
}

inline auto PatchParams::atlasToView(Common::Vec2i xy, const Common::Mat3x3i &m) noexcept
    -> Common::Vec2i {
  return {m(0, 0) * xy[0] + m(0, 1) * xy[1] + m(0, 2),  //
          m(1, 0) * xy[0] + m(1, 1) * xy[1] + m(1, 2)}; //
}

inline auto PatchParams::viewToAtlas(Common::Vec2i uv, const Common::Mat3x3i &m) noexcept
    -> Common::Vec2i {
  // NOTE(BK): limit to lodX == lodY == 1
  LIMITATION(m(2, 2) == 1);
  return atlasToView(uv, m);
}
} // namespace TMIV::MivBitstream
