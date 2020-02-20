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

#ifndef _TMIV_MIVBITSTREAM_IVACCESSUNITPARAMS_H_
#define _TMIV_MIVBITSTREAM_IVACCESSUNITPARAMS_H_

#include <TMIV/MivBitstream/AtlasTileGroupLayerRBSP.h>
#include <TMIV/MivBitstream/IvSequenceParams.h>

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Vector.h>

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// PatchParams is the in-memory representation of PatchDataUnit (PDU). The PDU is not suitable for
// in-memory use because of the delta coding and quantization of some of the fields.
struct PatchParams {
  // TODO(BK): Have a PatchParamsVector per atlas
  std::uint8_t vuhAtlasId{};

  auto pdu2dPos() const noexcept;
  auto pdu2dSize() const noexcept;
  auto pduViewPos() const noexcept;
  auto pduViewSize() const noexcept;
  auto pduDepthStart() const noexcept;
  auto pduDepthEnd() const noexcept;
  auto pduViewId() const noexcept;
  auto pduOrientationIndex() const noexcept;
  auto pduEntityId() const noexcept;
  auto pduDepthOccMapThreshold() const noexcept;

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
  bool isRotated() const;

  bool operator==(const PatchParams &other) const;
  bool operator!=(const PatchParams &other) const { return !operator==(other); };

private:
  Common::Vec2i m_pdu2dPos;
  Common::Vec2i m_pdu2dSize;
  Common::Vec2i m_pduViewPos;
  std::optional<std::uint16_t> m_pduDepthStart;
  std::optional<std::uint16_t> m_pduDepthEnd;
  std::uint16_t m_pduViewId{};
  FlexiblePatchOrientation m_pduOrientationIndex{};
  std::optional<std::uint16_t> m_pduEntityId;
  std::optional<std::uint16_t> m_pduDepthOccMapThreshold;
};

using PatchParamsVector = std::vector<PatchParams>;

struct AtlasParamsList : public PatchParamsVector {
  bool omafV1CompatibleFlag{};
  std::optional<std::vector<unsigned>> groupIds;
  Common::SizeVector atlasSizes;
  std::vector<bool> depthOccupancyParamsPresentFlags;

  void setAtlasParamsVector(PatchParamsVector x) { PatchParamsVector::operator=(move(x)); }

  friend std::ostream &operator<<(std::ostream &, const AtlasParamsList &);
  bool operator==(const AtlasParamsList &other) const;
  bool operator!=(const AtlasParamsList &other) const { return !operator==(other); }
};

// Pixel position conversion from atlas to/from view
Common::Vec2i viewToAtlas(Common::Vec2i viewPosition, const PatchParams &patch);
Common::Vec2i atlasToView(Common::Vec2i atlasPosition, const PatchParams &patch);

struct IvAccessUnitParams {
  AtlasParamsList atlasParamsList;

  friend std::ostream &operator<<(std::ostream &, const IvAccessUnitParams &);
  bool operator==(const IvAccessUnitParams &other) const;
  bool operator!=(const IvAccessUnitParams &other) const { return !operator==(other); }
};
} // namespace TMIV::MivBitstream

#include "IvAccessUnitParams.hpp"

#endif
