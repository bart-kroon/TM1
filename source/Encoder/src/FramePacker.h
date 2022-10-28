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

#ifndef TMIV_ENCODER_FRAMEPACK_H
#define TMIV_ENCODER_FRAMEPACK_H

#include <TMIV/Common/Frame.h>
#include <TMIV/Encoder/EncoderParams.h>

namespace TMIV::Encoder {
class FramePacker {
public:
  auto setPackingInformation(EncoderParams params, bool geometryPacking = false)
      -> const EncoderParams &;
  void packFrame(Common::V3cFrameList &frame, uint32_t bitDepth, bool geometryPacking = false);

private:
  struct RegionCounts {
    uint8_t attr{};
    uint8_t geo{};
    uint8_t occ{};
  };

  struct RegionSizes {
    Common::Vec2i frame{0, 0};
    Common::Vec2i geo{0, 0};
    Common::Vec2i occ{0, 0};
    Common::Vec2i pac{0, 0};
  };
  [[nodiscard]] auto packAtlasFrame(const Common::V3cFrame &frame, uint8_t atlasIdx,
                                    uint32_t bitDepth, bool geometryPacking = false) const
      -> Common::V3cFrame;

  void combinePlanes(size_t atlasIdx, const Common::Frame<> &atlasTexture);
  void extractScaledGeometry(size_t atlasIdx, const Common::heap::Matrix<uint16_t> &planeDepth);
  void updatePinOccupancyInformation(MivBitstream::AtlasId atlasId);
  void updateVideoPresentFlags(MivBitstream::AtlasId atlasId, bool geometryPacking = false);
  auto computeOccupancySizeAndRegionCount(size_t atlasIdx, bool geometryPacking = false) -> uint8_t;
  auto computeGeometrySizeAndRegionCount(size_t atlasIdx, bool geometryPacking = false) -> uint8_t;
  void updatePinGeometryInformation(MivBitstream::AtlasId atlasId);
  void updatePinAttributeInformation(MivBitstream::AtlasId atlasId);
  void setAttributePinRegion(size_t i, const Common::Vec2i &frameSize);
  void setGeometryPinRegion(size_t i, size_t atlasIdx, const RegionCounts &regionCounts);
  void setOccupancyPinRegion(size_t i, size_t atlasIdx, const RegionCounts &regionCounts);
  void updatePinRegionInformation(size_t i);
  void setGeoPckGeometryPinRegion(size_t i, size_t atlasIdx, const RegionCounts &regionCounts);
  void setGeoPckOccupancyPinRegion(size_t i, size_t atlasIdx, const RegionCounts &regionCounts);
  std::vector<RegionSizes> m_regionSizes{};
  EncoderParams m_params;
  MivBitstream::PackingInformation m_packingInformation{};
  MivBitstream::PinRegion m_pinRegion{};
};
} // namespace TMIV::Encoder

#endif
