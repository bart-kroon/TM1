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
#error "Include the .h, not the .hpp"
#endif

namespace TMIV::MivBitstream {
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

constexpr auto PatchParams::atlasPatchEntityId() const noexcept { return m_atlasPatchEntityId; }

constexpr auto PatchParams::atlasPatchDepthOccMapThreshold() const noexcept {
  return m_atlasPatchDepthOccMapThreshold;
}

inline auto PatchParams::atlasPatchAttributeOffset() const noexcept {
  VERIFY_MIVBITSTREAM(m_atlasPatchAttributeOffset.has_value());
  return *m_atlasPatchAttributeOffset;
}

constexpr auto PatchParams::pduInpaintFlag() const noexcept {
  return m_pduInpaintFlag;
}

constexpr auto PatchParams::atlasPatch2dPosX(std::uint32_t value) noexcept -> PatchParams & {
  m_atlasPatch2dPosX = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch2dPosY(std::uint32_t value) noexcept -> PatchParams & {
  m_atlasPatch2dPosY = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch2dSizeX(std::uint32_t value) noexcept -> PatchParams & {
  m_atlasPatch2dSizeX = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch2dSizeY(std::uint32_t value) noexcept -> PatchParams & {
  m_atlasPatch2dSizeY = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch3dOffsetU(std::uint32_t value) noexcept -> PatchParams & {
  m_atlasPatch3dOffsetU = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch3dOffsetV(std::uint32_t value) noexcept -> PatchParams & {
  m_atlasPatch3dOffsetV = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch3dOffsetD(std::uint32_t value) noexcept -> PatchParams & {
  m_atlasPatch3dOffsetD = value;
  return *this;
}

constexpr auto PatchParams::atlasPatch3dRangeD(std::uint32_t value) noexcept -> PatchParams & {
  m_atlasPatch3dRangeD = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchProjectionId(std::uint16_t value) noexcept -> PatchParams & {
  m_atlasPatchProjectionId = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchOrientationIndex(FlexiblePatchOrientation value) noexcept
    -> PatchParams & {
  m_atlasPatchOrientationIndex = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchEntityId(std::uint16_t value) noexcept -> PatchParams & {
  m_atlasPatchEntityId = value;
  return *this;
}

constexpr auto PatchParams::atlasPatchDepthOccMapThreshold(std::uint32_t value) noexcept
    -> PatchParams & {
  m_atlasPatchDepthOccMapThreshold = value;
  return *this;
}

inline auto PatchParams::atlasPatchAttributeOffset(Common::Vec3i value) noexcept -> PatchParams & {
  m_atlasPatchAttributeOffset = value;
  return *this;
}

constexpr auto& PatchParams::pduInpaintFlag(bool value) noexcept {
    m_pduInpaintFlag = value;
    return *this;
}

inline auto PatchParams::operator==(const PatchParams &other) const -> bool {
  return atlasId == other.atlasId && atlasPatch2dPosX() == other.atlasPatch2dPosX() &&
         atlasPatch2dPosY() == other.atlasPatch2dPosY() &&
         atlasPatch2dSizeX() == other.atlasPatch2dSizeX() &&
         atlasPatch2dSizeY() == other.atlasPatch2dSizeY() &&
         atlasPatch3dOffsetU() == other.atlasPatch3dOffsetU() &&
         atlasPatch3dOffsetV() == other.atlasPatch3dOffsetV() &&
         atlasPatch3dOffsetD() == other.atlasPatch3dOffsetD() &&
         atlasPatch3dRangeD() == other.atlasPatch3dRangeD() &&
         atlasPatchProjectionId() == other.atlasPatchProjectionId() &&
         atlasPatchOrientationIndex() == other.atlasPatchOrientationIndex() &&
         atlasPatchEntityId() == other.atlasPatchEntityId() &&
         atlasPatchDepthOccMapThreshold() == other.atlasPatchDepthOccMapThreshold() &&
         atlasPatchAttributeOffset() == other.atlasPatchAttributeOffset() &&
         pduInpaintFlag() == other.pduInpaintFlag();
}

constexpr auto PatchParams::isRotated() const noexcept {
  assert(atlasPatchOrientationIndex() <= FlexiblePatchOrientation::FPO_MROT180);

  return atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_ROT90 ||
         atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_SWAP ||
         atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_ROT270 ||
         atlasPatchOrientationIndex() == FlexiblePatchOrientation::FPO_MROT90;
}

constexpr decltype(auto) PatchParams::atlasPatch3dSizeU() const noexcept {
  return isRotated() ? atlasPatch2dSizeY() : atlasPatch2dSizeX();
}

constexpr decltype(auto) PatchParams::atlasPatch3dSizeV() const noexcept {
  return isRotated() ? atlasPatch2dSizeX() : atlasPatch2dSizeY();
}

constexpr decltype(auto) PatchParams::atlasPatch3dSizeU(std::uint32_t value) noexcept {
  return isRotated() ? atlasPatch2dSizeY(value) : atlasPatch2dSizeX(value);
}

constexpr decltype(auto) PatchParams::atlasPatch3dSizeV(std::uint32_t value) noexcept {
  return isRotated() ? atlasPatch2dSizeX(value) : atlasPatch2dSizeY(value);
}

inline auto PatchParams::viewToAtlas(Common::Vec2i viewPosition) const -> Common::Vec2i {
  const auto w = static_cast<int>(atlasPatch3dSizeU());
  const auto h = static_cast<int>(atlasPatch3dSizeV());
  const auto xM = static_cast<int>(atlasPatch3dOffsetU());
  const auto yM = static_cast<int>(atlasPatch3dOffsetV());
  const auto xP = static_cast<int>(atlasPatch2dPosX());
  const auto yP = static_cast<int>(atlasPatch2dPosY());
  const auto x = viewPosition.x();
  const auto y = viewPosition.y();

  switch (atlasPatchOrientationIndex()) {
  case FlexiblePatchOrientation::FPO_NULL: // (x, y)
    return {x - xM + xP, y - yM + yP};
  case FlexiblePatchOrientation::FPO_SWAP: // (y, x)
    return {y - yM + xP, x - xM + yP};
  case FlexiblePatchOrientation::FPO_ROT90: // (-y, x)
    return {-y + yM + xP + h - 1, x - xM + yP};
  case FlexiblePatchOrientation::FPO_ROT180: // (-x, -y)
    return {-x + xM + xP + w - 1, -y + yM + yP + h - 1};
  case FlexiblePatchOrientation::FPO_ROT270: // (y, -x)
    return {y - yM + xP, -x + xM + yP + w - 1};
  case FlexiblePatchOrientation::FPO_MIRROR: // (-x, y)
    return {-x + xM + xP + w - 1, y - yM + yP};
  case FlexiblePatchOrientation::FPO_MROT90: // (-y, -x)
    return {-y + yM + xP + h - 1, -x + xM + yP + w - 1};
  case FlexiblePatchOrientation::FPO_MROT180: // (x, -y)
    return {x - xM + xP, -y + yM + yP + h - 1};
  default:
    abort();
  }
}

inline auto PatchParams::atlasToView(Common::Vec2i atlasPosition) const -> Common::Vec2i {
  const auto w = static_cast<int>(atlasPatch3dSizeU());
  const auto h = static_cast<int>(atlasPatch3dSizeV());
  const auto xM = static_cast<int>(atlasPatch3dOffsetU());
  const auto yM = static_cast<int>(atlasPatch3dOffsetV());
  const auto xP = static_cast<int>(atlasPatch2dPosX());
  const auto yP = static_cast<int>(atlasPatch2dPosY());
  const auto x = atlasPosition.x();
  const auto y = atlasPosition.y();

  switch (atlasPatchOrientationIndex()) {
  case FlexiblePatchOrientation::FPO_NULL: // (x, y)
    return {x - xP + xM, y - yP + yM};
  case FlexiblePatchOrientation::FPO_SWAP: // (y, x)
    return {y - yP + xM, x - xP + yM};
  case FlexiblePatchOrientation::FPO_ROT90: // (y, -x)
    return {y - yP + xM, -x + xP + yM + h - 1};
  case FlexiblePatchOrientation::FPO_ROT180: // (-x, -y)
    return {-x + xP + xM + w - 1, -y + yP + yM + h - 1};
  case FlexiblePatchOrientation::FPO_ROT270: // (-y, x)
    return {-y + yP + xM + w - 1, x - xP + yM};
  case FlexiblePatchOrientation::FPO_MIRROR: // (-x, y)
    return {-x + xP + xM + w - 1, y - yP + yM};
  case FlexiblePatchOrientation::FPO_MROT90: // (-y, -x)
    return {-y + yP + xM + w - 1, -x + xP + yM + h - 1};
  case FlexiblePatchOrientation::FPO_MROT180: // (x, -y)
    return {x - xP + xM, -y + yP + yM + h - 1};
  default:
    abort();
  }
}
} // namespace TMIV::MivBitstream
