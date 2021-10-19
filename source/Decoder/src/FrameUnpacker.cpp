/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#include <TMIV/Decoder/FrameUnpacker.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <utility>

namespace TMIV::Decoder {
void FrameUnpacker::readRegionParams(const TMIV::MivBitstream::PackingInformation &PackingInfo) {
  m_numRegions = PackingInfo.pin_regions_count_minus1() + 1;
  m_numAttributes = PackingInfo.pin_attribute_count();
  m_numPartitions = PackingInfo.pin_attribute_dimension_partitions_minus1(0) + 1;
  LIMITATION(m_numMaps == 1 && m_numPartitions == 1 && m_numAttributes == 1);

  m_regionParams.regionTypeId = std::vector<MivBitstream::VuhUnitType>(m_numRegions);
  m_regionParams.regionPackedOffsetX = std::vector<uint16_t>(m_numRegions);
  m_regionParams.regionPackedOffsetY = std::vector<uint16_t>(m_numRegions);
  m_regionParams.regionWidth = std::vector<uint16_t>(m_numRegions);
  m_regionParams.regionHeight = std::vector<uint16_t>(m_numRegions);
  m_regionParams.regionUnpackedOffsetX = std::vector<uint16_t>(m_numRegions);
  m_regionParams.regionUnpackedOffsetY = std::vector<uint16_t>(m_numRegions);
  m_regionParams.regionRotationFlag = std::vector<bool>(m_numRegions);
  m_regionParams.regionAuxilaryDataFlag = std::vector<bool>(m_numRegions);
  m_regionParams.regionAttrTypeID = std::vector<uint8_t>(m_numRegions);

  for (auto i = 0; i < m_numRegions; i++) {
    m_regionParams.regionTypeId[i] = PackingInfo.pinRegionTypeId(i);
    m_regionParams.regionPackedOffsetX[i] = PackingInfo.pin_region_top_left_x(i);
    m_regionParams.regionPackedOffsetY[i] = PackingInfo.pin_region_top_left_y(i);
    m_regionParams.regionWidth[i] = PackingInfo.pin_region_width_minus1(i) + 1;
    m_regionParams.regionHeight[i] = PackingInfo.pin_region_height_minus1(i) + 1;
    m_regionParams.regionUnpackedOffsetX[i] = PackingInfo.pin_region_unpack_top_left_x(i);
    m_regionParams.regionUnpackedOffsetY[i] = PackingInfo.pin_region_unpack_top_left_y(i);
    m_regionParams.regionRotationFlag[i] = PackingInfo.pin_region_rotation_flag(i);
    if (m_regionParams.regionTypeId[i] == MivBitstream::VuhUnitType::V3C_AVD ||
        m_regionParams.regionTypeId[i] == MivBitstream::VuhUnitType::V3C_GVD) {
      m_regionParams.regionAuxilaryDataFlag[i] = PackingInfo.pin_region_auxiliary_data_flag(i);
    }
    if (m_regionParams.regionTypeId[i] == MivBitstream::VuhUnitType::V3C_AVD) {
      const auto type = PackingInfo.pin_attribute_type_id(PackingInfo.pin_region_attr_index(i));
      LIMITATION(type == TMIV::MivBitstream::AiAttributeTypeId::ATTR_TEXTURE);
      m_regionParams.regionAttrTypeID[i] = static_cast<uint8_t>(type);
    }
  }
}

void FrameUnpacker::calculateUnpackSizes() {
  for (auto i = 0; i < m_numRegions; i++) {
    const Common::Vec2u componentSize = [&]() {
      if (m_regionParams.regionRotationFlag[i]) {
        return Common::Vec2u{static_cast<uint32_t>(m_regionParams.regionUnpackedOffsetX[i] +
                                                   m_regionParams.regionHeight[i]),
                             static_cast<uint32_t>(m_regionParams.regionUnpackedOffsetY[i] +
                                                   m_regionParams.regionWidth[i])};
      }
      return Common::Vec2u{static_cast<uint32_t>(m_regionParams.regionUnpackedOffsetX[i] +
                                                 m_regionParams.regionWidth[i]),
                           static_cast<uint32_t>(m_regionParams.regionUnpackedOffsetY[i] +
                                                 m_regionParams.regionHeight[i])};
    }();

    if (m_regionParams.regionTypeId[i] == MivBitstream::VuhUnitType::V3C_OVD) {
      m_unpackSizes.unpckOccSize.x() = std::max(componentSize.x(), m_unpackSizes.unpckOccSize.x());
      m_unpackSizes.unpckOccSize.y() = std::max(componentSize.y(), m_unpackSizes.unpckOccSize.y());
    } else if (m_regionParams.regionTypeId[i] == MivBitstream::VuhUnitType::V3C_GVD) {
      LIMITATION(!m_regionParams.regionAuxilaryDataFlag[i]);
      m_unpackSizes.unpckGeoSize.x() = std::max(componentSize.x(), m_unpackSizes.unpckGeoSize.x());
      m_unpackSizes.unpckGeoSize.y() = std::max(componentSize.y(), m_unpackSizes.unpckGeoSize.y());
    } else if (m_regionParams.regionTypeId[i] == MivBitstream::VuhUnitType::V3C_AVD) {
      LIMITATION(!m_regionParams.regionAuxilaryDataFlag[i]);
      m_unpackSizes.unpckAttrSize.x() =
          std::max(componentSize.x(), m_unpackSizes.unpckAttrSize.x());
      m_unpackSizes.unpckAttrSize.y() =
          std::max(componentSize.y(), m_unpackSizes.unpckAttrSize.y());
    }
  }
}

void FrameUnpacker::initializeUnpackComponents(MivBitstream::AtlasAccessUnit &atlas) const {
  // TODO(#397): Magic bit depth
  atlas.attrFrame.createYuv444({static_cast<int32_t>(m_unpackSizes.unpckAttrSize.x()),
                                static_cast<int32_t>(m_unpackSizes.unpckAttrSize.y())},
                               10);

  // TODO(#397): Magic bit depth
  atlas.decGeoFrame.createY({static_cast<int32_t>(m_unpackSizes.unpckGeoSize.x()),
                             static_cast<int32_t>(m_unpackSizes.unpckGeoSize.y())},
                            10);

  // TODO(#397): Magic bit depth
  atlas.decOccFrame.createY({static_cast<int32_t>(m_unpackSizes.unpckOccSize.x()),
                             static_cast<int32_t>(m_unpackSizes.unpckOccSize.y())},
                            10);
}

void FrameUnpacker::extractUnpackedComponents(MivBitstream::AtlasAccessUnit &atlas) {
  for (auto i = 0; i < m_numRegions; i++) {
    for (auto w = 0; w < m_regionParams.regionWidth[i]; w++) {
      for (auto h = 0; h < m_regionParams.regionHeight[i]; h++) {
        const Common::Vec2u p = {static_cast<uint32_t>(m_regionParams.regionPackedOffsetX[i] + w),
                                 static_cast<uint32_t>(m_regionParams.regionPackedOffsetY[i] + h)};
        const Common::Vec2u u = [&]() {
          if (m_regionParams.regionRotationFlag[i]) {
            return Common::Vec2u{
                static_cast<uint32_t>(m_regionParams.regionUnpackedOffsetX[i] + h),
                static_cast<uint32_t>(m_regionParams.regionUnpackedOffsetY[i] + w)};
          }
          return Common::Vec2u{static_cast<uint32_t>(m_regionParams.regionUnpackedOffsetX[i] + w),
                               static_cast<uint32_t>(m_regionParams.regionUnpackedOffsetY[i] + h)};
        }();

        if (m_regionParams.regionTypeId[i] == MivBitstream::VuhUnitType::V3C_OVD) {
          atlas.decOccFrame.getPlane(0)(u.y(), u.x()) = atlas.decPacFrame.getPlane(0)(p.y(), p.x());
        } else if (m_regionParams.regionTypeId[i] == MivBitstream::VuhUnitType::V3C_GVD) {
          LIMITATION(!m_regionParams.regionAuxilaryDataFlag[i]);
          atlas.decGeoFrame.getPlane(0)(u.y(), u.x()) = atlas.decPacFrame.getPlane(0)(p.y(), p.x());
        } else if (m_regionParams.regionTypeId[i] == MivBitstream::VuhUnitType::V3C_AVD) {
          LIMITATION(!m_regionParams.regionAuxilaryDataFlag[i]);
          for (auto c = 0; c < 3; c++) {
            atlas.attrFrame.getPlane(c)(u.y(), u.x()) = atlas.decPacFrame.getPlane(c)(p.y(), p.x());
          }
        }
      }
    }
  }
}

void FrameUnpacker::inplaceUnpack(MivBitstream::AccessUnit &frame) {
  // Implementation is done according to 23090-5_PDIS_d11
  for (auto k = 0; k <= frame.vps.vps_atlas_count_minus1(); ++k) {
    // B.4.1 General
    const auto atlasId = frame.vps.vps_atlas_id(k);
    m_numMaps = frame.vps.vps_map_count_minus1(atlasId) + 1;
    readRegionParams(frame.vps.packing_information(atlasId));
    // B.4.2 Calculate unpacked video component resolution
    calculateUnpackSizes();
    // B.4.3 Initializing unpacked video component frame
    initializeUnpackComponents(frame.atlas[k]);
    // B.4.4 Copying data from packed regions to unpacked video component frames process
    extractUnpackedComponents(frame.atlas[k]);
  }
}

} // namespace TMIV::Decoder
