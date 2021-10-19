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

#include <TMIV/Encoder/FramePacker.h>

namespace TMIV::Encoder {
void FramePacker::constructFramePack(Common::MVD10Frame &frame) {
  uint8_t atlasIdx{};

  for (auto &atlas : frame) {
    atlas.framePack = constructPackedAtlasFrame(frame[atlasIdx], atlasIdx);
    atlasIdx++;
  }
}

auto FramePacker::constructPackedAtlasFrame(const Common::TextureDepth10Frame &frame,
                                            uint8_t atlasIdx) const -> Common::FramePack10Frame {
  // TODO(#397): Use a configuration parameter to determine the packed frame bit depth
  auto result =
      Common::FramePack10Frame::yuv420(m_regionSizes[atlasIdx].pac, frame.texture.getBitDepth());
  result.fillNeutral();

  for (uint8_t regionIdx = 0; regionIdx <= m_packingInformation.pin_regions_count_minus1();
       ++regionIdx) {
    const auto topLeftX = m_packingInformation.pin_region_top_left_x(regionIdx);
    const auto topLeftY = m_packingInformation.pin_region_top_left_y(regionIdx);
    const auto width = m_packingInformation.pin_region_width_minus1(regionIdx) + 1;
    const auto height = m_packingInformation.pin_region_height_minus1(regionIdx) + 1;
    const auto unpackTopLeftX = m_packingInformation.pin_region_unpack_top_left_x(regionIdx);
    const auto unpackTopLeftY = m_packingInformation.pin_region_unpack_top_left_y(regionIdx);
    const auto rotation = m_packingInformation.pin_region_rotation_flag(regionIdx);

    for (int32_t i = 0; i < height; ++i) {
      for (int32_t j = 0; j < width; ++j) {
        const auto px = topLeftX + j;
        const auto py = topLeftY + i;

        const auto ux = unpackTopLeftX + (rotation ? i : j);
        const auto uy = unpackTopLeftY + (rotation ? j : i);

        switch (m_packingInformation.pinRegionTypeId(regionIdx)) {
        case MivBitstream::VuhUnitType::V3C_OVD:
          result.getPlane(0)(py, px) = frame.occupancy.getPlane(0)(uy, ux);
          break;
        case MivBitstream::VuhUnitType::V3C_GVD:
          result.getPlane(0)(py, px) = frame.depth.getPlane(0)(uy, ux);
          break;
        case MivBitstream::VuhUnitType::V3C_AVD: {
          const auto attrIdx = m_packingInformation.pin_region_attr_index(regionIdx);
          const auto attrTypeId = m_packingInformation.pin_attribute_type_id(attrIdx);

          switch (attrTypeId) {
          case MivBitstream::AiAttributeTypeId::ATTR_TEXTURE:
            result.getPlane(0)(py, px) = frame.texture.getPlane(0)(uy, ux);
            result.getPlane(1)(py / 2, px / 2) = frame.texture.getPlane(1)(uy / 2, ux / 2);
            result.getPlane(2)(py / 2, px / 2) = frame.texture.getPlane(2)(uy / 2, ux / 2);
            break;
          case MivBitstream::AiAttributeTypeId::ATTR_TRANSPARENCY:
            result.getPlane(0)(py, px) = frame.transparency.getPlane(0)(uy, ux);
            break;
          default:
            UNREACHABLE;
          }
          break;
        }
        default:
          UNREACHABLE;
        }
      }
    }
  }

  return result;
}

void FramePacker::updateVideoPresentFlags(MivBitstream::AtlasId atlasId) {
  m_packingInformation
      .pin_occupancy_present_flag(m_params.vps.vps_occupancy_video_present_flag(atlasId))
      .pin_geometry_present_flag(m_params.vps.vps_geometry_video_present_flag(atlasId))
      .pin_attribute_present_flag(m_params.vps.vps_attribute_video_present_flag(atlasId));
  m_params.vps.vps_occupancy_video_present_flag(atlasId, false)
      .vps_geometry_video_present_flag(atlasId, false)
      .vps_attribute_video_present_flag(atlasId, false);
}

void FramePacker::updatePinOccupancyInformation(MivBitstream::AtlasId atlasId) {
  m_packingInformation.pin_occupancy_2d_bit_depth_minus1(
      m_params.vps.occupancy_information(atlasId).oi_occupancy_2d_bit_depth_minus1());
  m_packingInformation.pin_occupancy_MSB_align_flag(
      m_params.vps.occupancy_information(atlasId).oi_occupancy_MSB_align_flag());
  m_packingInformation.pin_lossy_occupancy_compression_threshold(
      m_params.vps.occupancy_information(atlasId).oi_lossy_occupancy_compression_threshold());
}

auto FramePacker::computeOccupancySizeAndRegionCount(size_t atlasIdx) -> uint8_t {
  m_regionSizes[atlasIdx].occ = m_regionSizes[atlasIdx].frame;
  const auto &asmeAtlas = m_params.atlas[atlasIdx].asps.asps_miv_extension();
  const bool occScaled = asmeAtlas.asme_occupancy_scale_enabled_flag();
  if (occScaled) {
    m_regionSizes[atlasIdx].occ.x() =
        m_regionSizes[atlasIdx].frame.x() / (asmeAtlas.asme_occupancy_scale_factor_x_minus1() + 1);
    m_regionSizes[atlasIdx].occ.y() =
        m_regionSizes[atlasIdx].frame.y() / (asmeAtlas.asme_occupancy_scale_factor_y_minus1() + 1);
  }
  return static_cast<uint8_t>(m_regionSizes[atlasIdx].frame.x() / m_regionSizes[atlasIdx].occ.x());
}

auto FramePacker::computeGeometrySizeAndRegionCount(size_t atlasIdx) -> uint8_t {
  m_regionSizes[atlasIdx].geo = m_regionSizes[atlasIdx].frame;
  const auto &asmeAtlas = m_params.atlas[atlasIdx].asps.asps_miv_extension();
  const bool geoScaled = asmeAtlas.asme_geometry_scale_enabled_flag();
  if (geoScaled) {
    m_regionSizes[atlasIdx].geo.x() =
        m_regionSizes[atlasIdx].frame.x() / (asmeAtlas.asme_geometry_scale_factor_x_minus1() + 1);
    m_regionSizes[atlasIdx].geo.y() =
        m_regionSizes[atlasIdx].frame.y() / (asmeAtlas.asme_geometry_scale_factor_y_minus1() + 1);
  }
  return static_cast<uint8_t>(m_regionSizes[atlasIdx].frame.x() / m_regionSizes[atlasIdx].geo.x());
}

void FramePacker::updatePinGeometryInformation(MivBitstream::AtlasId atlasId) {
  m_packingInformation.pin_geometry_2d_bit_depth_minus1(
      m_params.vps.geometry_information(atlasId).gi_geometry_2d_bit_depth_minus1());
  m_packingInformation.pin_geometry_MSB_align_flag(
      m_params.vps.geometry_information(atlasId).gi_geometry_MSB_align_flag());
  m_packingInformation.pin_geometry_3d_coordinates_bit_depth_minus1(
      m_params.vps.geometry_information(atlasId).gi_geometry_3d_coordinates_bit_depth_minus1());
}

void FramePacker::updatePinAttributeInformation(MivBitstream::AtlasId atlasId) {
  m_packingInformation.pin_attribute_count(
      m_params.vps.attribute_information(atlasId).ai_attribute_count());
  for (uint8_t i = 0; i < m_params.vps.attribute_information(atlasId).ai_attribute_count(); i++) {
    m_packingInformation.pin_attribute_type_id(i, MivBitstream::AiAttributeTypeId::ATTR_TEXTURE);
    m_packingInformation.pin_attribute_2d_bit_depth_minus1(
        i, m_params.vps.attribute_information(atlasId).ai_attribute_2d_bit_depth_minus1(i));
    m_packingInformation.pin_attribute_MSB_align_flag(
        0, m_params.vps.attribute_information(atlasId).ai_attribute_MSB_align_flag(i));
    m_packingInformation.pin_attribute_map_absolute_coding_persistence_flag(i, false);
    m_packingInformation.pin_attribute_dimension_minus1(i, 0);
  }
}

void FramePacker::updatePinRegionInformation(size_t i) {
  m_packingInformation.pin_region_tile_id(i, 0);
  m_packingInformation.pin_region_type_id_minus2(i, m_pinRegion.pin_region_type_id_minus2);
  m_packingInformation.pin_region_top_left_x(i, m_pinRegion.pin_region_top_left_x);
  m_packingInformation.pin_region_top_left_y(i, m_pinRegion.pin_region_top_left_y);
  m_packingInformation.pin_region_width_minus1(i, m_pinRegion.pin_region_width_minus1);
  m_packingInformation.pin_region_height_minus1(i, m_pinRegion.pin_region_height_minus1);
  m_packingInformation.pin_region_unpack_top_left_x(i, m_pinRegion.pin_region_unpack_top_left_x);
  m_packingInformation.pin_region_unpack_top_left_y(i, m_pinRegion.pin_region_unpack_top_left_y);
  m_packingInformation.pin_region_rotation_flag(i, false);

  if (m_pinRegion.pinRegionTypeId() == MivBitstream::VuhUnitType::V3C_AVD ||
      m_pinRegion.pinRegionTypeId() == MivBitstream::VuhUnitType::V3C_GVD) {
    m_packingInformation.pin_region_map_index(i, 0);
    m_packingInformation.pin_region_auxiliary_data_flag(i, false);
  }
  if (m_pinRegion.pinRegionTypeId() == MivBitstream::VuhUnitType::V3C_AVD) {
    m_packingInformation.pin_region_attr_index(i, 0);
  }
}

void FramePacker::setAttributePinRegion(size_t i, const Common::Vec2i &frameSize) {
  m_pinRegion.pin_region_type_id_minus2 = static_cast<uint16_t>(2);
  m_pinRegion.pin_region_top_left_x = 0;
  m_pinRegion.pin_region_top_left_y = static_cast<uint16_t>(frameSize.y() * i);
  m_pinRegion.pin_region_width_minus1 = static_cast<uint16_t>(frameSize.x() - 1);
  m_pinRegion.pin_region_height_minus1 = static_cast<uint16_t>(frameSize.y() - 1);
  m_pinRegion.pin_region_unpack_top_left_x = 0;
  m_pinRegion.pin_region_unpack_top_left_y = 0;
}

void FramePacker::setGeometryPinRegion(size_t i, size_t atlasIdx,
                                       const RegionCounts &regionCounts) {
  m_pinRegion.pin_region_type_id_minus2 = static_cast<uint16_t>(1);
  m_pinRegion.pin_region_top_left_x =
      static_cast<uint16_t>(m_regionSizes[atlasIdx].geo.x() * (i - regionCounts.attr));
  m_pinRegion.pin_region_top_left_y =
      static_cast<uint16_t>(m_regionSizes[atlasIdx].frame.y() * regionCounts.attr);
  m_pinRegion.pin_region_width_minus1 = static_cast<uint16_t>(m_regionSizes[atlasIdx].geo.x() - 1);
  m_pinRegion.pin_region_height_minus1 =
      static_cast<uint16_t>((m_regionSizes[atlasIdx].geo.y() / regionCounts.geo) - 1);
  m_pinRegion.pin_region_unpack_top_left_x = 0;
  m_pinRegion.pin_region_unpack_top_left_y = static_cast<uint16_t>(
      (i - regionCounts.attr) * (m_regionSizes[atlasIdx].geo.y() / regionCounts.geo));
}

void FramePacker::setOccupancyPinRegion(size_t i, size_t atlasIdx,
                                        const RegionCounts &regionCounts) {
  m_pinRegion.pin_region_type_id_minus2 = static_cast<uint16_t>(0);
  m_pinRegion.pin_region_top_left_x = static_cast<uint16_t>(
      m_regionSizes[atlasIdx].occ.x() * (i - (regionCounts.attr + regionCounts.geo)));
  if (regionCounts.geo != 0) {
    m_pinRegion.pin_region_top_left_y =
        static_cast<uint16_t>(m_regionSizes[atlasIdx].frame.y() * regionCounts.attr +
                              (m_regionSizes[atlasIdx].geo.y() / regionCounts.geo));
  } else {
    m_pinRegion.pin_region_top_left_y =
        static_cast<uint16_t>(m_regionSizes[atlasIdx].frame.y() * regionCounts.attr);
  }
  m_pinRegion.pin_region_width_minus1 = static_cast<uint16_t>(m_regionSizes[atlasIdx].occ.x() - 1);
  m_pinRegion.pin_region_height_minus1 =
      static_cast<uint16_t>((m_regionSizes[atlasIdx].occ.y() / regionCounts.occ) - 1);
  m_pinRegion.pin_region_unpack_top_left_x = static_cast<uint16_t>(0);
  m_pinRegion.pin_region_unpack_top_left_y =
      static_cast<uint16_t>((i - (regionCounts.attr + regionCounts.geo)) *
                            (m_regionSizes[atlasIdx].occ.y() / regionCounts.occ));
}

auto FramePacker::setPackingInformation(EncoderParams params) -> const EncoderParams & {
  // Current implementation is limited to texture attribute, geometry, and occupancy
  m_params = std::move(params);

  m_params.vps.vps_packing_information_present_flag(true);
  m_regionSizes.clear();
  for (size_t atlasIdx = 0; atlasIdx <= m_params.vps.vps_atlas_count_minus1(); atlasIdx++) {
    const auto atlasId = m_params.vps.vps_atlas_id(atlasIdx);
    updateVideoPresentFlags(atlasId);

    RegionCounts regionCounts{};
    m_regionSizes.push_back(RegionSizes{});
    m_regionSizes[atlasIdx].frame = {m_params.vps.vps_frame_width(atlasId),
                                     m_params.vps.vps_frame_height(atlasId)};
    m_regionSizes[atlasIdx].pac.x() = m_regionSizes[atlasIdx].frame.x();
    if (m_packingInformation.pin_occupancy_present_flag()) {
      updatePinOccupancyInformation(atlasId);
      regionCounts.occ = computeOccupancySizeAndRegionCount(atlasIdx);
      m_regionSizes[atlasIdx].pac.y() += m_regionSizes[atlasIdx].occ.y() / regionCounts.occ;
    }

    if (m_packingInformation.pin_geometry_present_flag()) {
      updatePinGeometryInformation(atlasId);
      regionCounts.geo = computeGeometrySizeAndRegionCount(atlasIdx);
      m_regionSizes[atlasIdx].pac.y() += m_regionSizes[atlasIdx].geo.y() / regionCounts.geo;
    }

    if (m_packingInformation.pin_attribute_present_flag()) {
      updatePinAttributeInformation(atlasId);
      regionCounts.attr = m_packingInformation.pin_attribute_count();
      m_regionSizes[atlasIdx].pac.y() += m_regionSizes[atlasIdx].frame.y() * regionCounts.attr;
    }

    const uint8_t regionsCountMinus1 = regionCounts.attr + regionCounts.geo + regionCounts.occ - 1;
    m_packingInformation.pin_regions_count_minus1(regionsCountMinus1);

    for (size_t i = 0; i <= regionsCountMinus1; i++) {
      if (i < regionCounts.attr) {
        setAttributePinRegion(i, m_regionSizes[atlasIdx].frame);
      } else if (i >= regionCounts.attr && i < (regionCounts.attr + regionCounts.geo)) {
        setGeometryPinRegion(i, atlasIdx, regionCounts);
      } else {
        setOccupancyPinRegion(i, atlasIdx, regionCounts);
      }

      updatePinRegionInformation(i);
    }
    m_params.vps.vps_packed_video_present_flag(atlasId, true);
    m_params.vps.packing_information(atlasId, m_packingInformation);
  }
  return m_params;
}
} // namespace TMIV::Encoder
