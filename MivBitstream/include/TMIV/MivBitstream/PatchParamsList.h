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

#include <TMIV/MivBitstream/AtlasTileGroupLayerRBSP.h>

#include <TMIV/Common/Vector.h>

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// PatchParams is the in-memory representation of PatchDataUnit (PDU). The PDU is not suitable for
// in-memory use because of the delta coding and quantization of some of the fields.
struct PatchParams {
  std::uint8_t vuhAtlasId{};

  [[nodiscard]] auto pdu2dPos() const noexcept;
  [[nodiscard]] auto pdu2dSize() const noexcept;
  [[nodiscard]] auto pduViewPos() const noexcept;
  [[nodiscard]] auto pduViewSize() const noexcept;
  [[nodiscard]] auto pduDepthStart() const noexcept;
  [[nodiscard]] auto pduDepthEnd() const noexcept;
  [[nodiscard]] auto pduViewId() const noexcept;
  [[nodiscard]] auto pduOrientationIndex() const noexcept;
  [[nodiscard]] auto pduEntityId() const noexcept;
  [[nodiscard]] auto pduDepthOccMapThreshold() const noexcept;

  auto pdu2dPos(const Common::Vec2i value) noexcept -> PatchParams &;
  auto pdu2dSize(const Common::Vec2i value) noexcept -> PatchParams &;
  auto pduViewSize(const Common::Vec2i value) noexcept -> PatchParams &;
  auto pduViewPos(const Common::Vec2i value) noexcept -> PatchParams &;
  auto pduDepthStart(const std::uint16_t value) noexcept -> PatchParams &;
  auto pduDepthEnd(const std::uint16_t value) noexcept -> PatchParams &;
  auto pduViewId(const std::uint16_t value) noexcept -> PatchParams &;
  auto pduOrientationIndex(const FlexiblePatchOrientation value) noexcept -> PatchParams &;
  auto pduEntityId(const std::uint16_t value) noexcept -> PatchParams &;
  auto pduDepthOccMapThreshold(const std::uint16_t value) noexcept -> PatchParams &;

  // Is the patch rotated such that width and height swap?
  [[nodiscard]] auto isRotated() const -> bool;

  // Pixel position conversion from atlas to/from view
  [[nodiscard]] auto viewToAtlas(Common::Vec2i viewPosition) const -> Common::Vec2i;
  [[nodiscard]] auto atlasToView(Common::Vec2i atlasPosition) const -> Common::Vec2i;

  auto operator==(const PatchParams &other) const -> bool;
  auto operator!=(const PatchParams &other) const -> bool { return !operator==(other); };

private:
  Common::Vec2i m_pdu2dPos;
  Common::Vec2i m_pduViewSize;
  Common::Vec2i m_pduViewPos;
  std::uint16_t m_pduDepthStart{};
  std::optional<std::uint16_t> m_pduDepthEnd;
  std::uint16_t m_pduViewId{};
  FlexiblePatchOrientation m_pduOrientationIndex{FlexiblePatchOrientation::FPO_INVALID};
  std::optional<std::uint16_t> m_pduEntityId;
  std::optional<std::uint16_t> m_pduDepthOccMapThreshold;
};

using PatchParamsList = std::vector<PatchParams>;
} // namespace TMIV::MivBitstream

#include "PatchParamsList.hpp"

#endif
