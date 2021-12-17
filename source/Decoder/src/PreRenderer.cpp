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

#include <TMIV/Decoder/PreRenderer.h>

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

namespace TMIV::Decoder {
PreRenderer::PreRenderer(const Common::Json &rootNode, const Common::Json &componentNode)
    : m_geometryScaler{rootNode, componentNode} {
  if (const auto &node = componentNode.optional("EntityDecodeRange")) {
    m_entityDecodeRange = node.asVec<uint32_t, 2>();
  }
}

void PreRenderer::preRenderFrame(MivBitstream::AccessUnit &frame) const {
  // ISO/IEC 23090-12 Annex A: profiles, tiers and levels
  checkRestrictions(frame);

  for (uint8_t atlasIdx = 0; atlasIdx <= frame.vps.vps_atlas_count_minus1(); ++atlasIdx) {
    const auto atlasId = frame.vps.vps_atlas_id(atlasIdx);
    auto &atlas = frame.atlas[atlasIdx];

    // ISO/IEC 23090-12 Annex B: post decoding
    unpackDecodedPackedVideo(frame.vps, atlasId, atlas);
    convertNominalFormat(frame.vps, atlasId, atlas);

    // ISO/IEC 23090-12 Annex H: rendering processes
    offsetTexture(frame.vps, atlasId, atlas);
    scaleGeometryVideo(frame.gup, atlas);
    reconstructOccupancy(frame.viewParamsList, atlas);
    filterEntities(atlas);
  }
}

void PreRenderer::checkRestrictions(const MivBitstream::AccessUnit &frame) {
  if (frame.vui) {
    if (frame.vui->vui_coordinate_system_parameters_present_flag()) {
      const auto &csp = frame.vui->coordinate_system_parameters();
      if (!csp.isOmafCas()) {
        throw std::runtime_error(
            "The VUI indicates that a coordinate axis system other than that of OMAF is used. "
            "The TMIV decoder/renderer is not yet able to convert between coordinate axis "
            "systems.");
      }
    }
  }
}

void PreRenderer::convertNominalFormat(const MivBitstream::V3cParameterSet &vps,
                                       MivBitstream::AtlasId atlasId,
                                       MivBitstream::AtlasAccessUnit &atlas) {
  if (!atlas.decOccFrame.empty()) {
    convertOccupancyNominalFormat(vps, atlasId, atlas, atlas.decOccFrame);
  } else if (!atlas.unpckOccFrame.empty()) {
    convertOccupancyNominalFormat(vps, atlasId, atlas, atlas.unpckOccFrame);
  }

  if (!atlas.decGeoFrame.empty()) {
    convertGeometryNominalFormat(vps, atlasId, atlas, atlas.decGeoFrame);
  } else if (!atlas.unpckGeoFrame.empty()) {
    convertGeometryNominalFormat(vps, atlasId, atlas, atlas.unpckGeoFrame);
  }

  if (!atlas.decAttrFrame.empty()) {
    convertAttributeNominalFormat(vps, atlasId, atlas, atlas.decAttrFrame);
  } else if (!atlas.unpckAttrFrame.empty()) {
    convertAttributeNominalFormat(vps, atlasId, atlas, atlas.unpckAttrFrame);
  }
}

void PreRenderer::convertOccupancyNominalFormat(const MivBitstream::V3cParameterSet &vps,
                                                MivBitstream::AtlasId atlasId,
                                                MivBitstream::AtlasAccessUnit &atlas,
                                                const Common::Frame<> &inFrame) {
  const auto videoWidthNF = vps.vps_frame_width(atlasId);
  const auto videoHeightNF = vps.vps_frame_height(atlasId);
  uint32_t occBitDepthNF{};
  bool occMSBAlignFlag{};
  uint8_t occThreshold{};

  if (vps.vps_occupancy_video_present_flag(atlasId)) {
    const auto &oi = vps.occupancy_information(atlasId);
    occBitDepthNF = oi.oi_occupancy_2d_bit_depth_minus1() + 1U;
    occMSBAlignFlag = oi.oi_occupancy_MSB_align_flag();
    occThreshold = oi.oi_lossy_occupancy_compression_threshold();
  } else {
    const auto &pin = vps.packing_information(atlasId);
    occBitDepthNF = pin.pin_occupancy_2d_bit_depth_minus1() + 1U;
    occMSBAlignFlag = pin.pin_occupancy_MSB_align_flag();
    occThreshold = pin.pin_lossy_occupancy_compression_threshold();
  }

  auto decOccFrameNBD = convertBitDepth(inFrame, occBitDepthNF, occMSBAlignFlag, 1);
  const auto decOccFrameNR =
      convertResolution(std::move(decOccFrameNBD), videoWidthNF, videoHeightNF);
  atlas.occFrameNF = thresholdOccupancy(decOccFrameNR, occThreshold);
}

void PreRenderer::convertGeometryNominalFormat(const MivBitstream::V3cParameterSet &vps,
                                               MivBitstream::AtlasId atlasId,
                                               MivBitstream::AtlasAccessUnit &atlas,
                                               const Common::Frame<> &inFrame) {
  auto videoWidthNF = vps.vps_frame_width(atlasId);
  auto videoHeightNF = vps.vps_frame_height(atlasId);

  if (vps.vps_miv_extension_present_flag() &&
      vps.vps_miv_extension().vme_geometry_scale_enabled_flag()) {
    const auto &asme = atlas.asps.asps_miv_extension();
    const auto asmeGeometryScaleFactorX = asme.asme_geometry_scale_factor_x_minus1() + int32_t{1};
    const auto asmeGeometryScaleFactorY = asme.asme_geometry_scale_factor_y_minus1() + int32_t{1};

    VERIFY_MIVBITSTREAM(videoWidthNF % asmeGeometryScaleFactorX == 0);
    VERIFY_MIVBITSTREAM(videoHeightNF % asmeGeometryScaleFactorY == 0);

    videoWidthNF /= asmeGeometryScaleFactorX;
    videoHeightNF /= asmeGeometryScaleFactorY;
  }

  uint32_t geoBitDepthNF{};
  bool geoMSBAlignFlag{};

  if (vps.vps_geometry_video_present_flag(atlasId)) {
    const auto &gi = vps.geometry_information(atlasId);
    geoBitDepthNF = gi.gi_geometry_2d_bit_depth_minus1() + 1U;
    geoMSBAlignFlag = gi.gi_geometry_MSB_align_flag();
  } else {
    const auto &pin = vps.packing_information(atlasId);
    geoBitDepthNF = pin.pin_geometry_2d_bit_depth_minus1() + 1U;
    geoMSBAlignFlag = pin.pin_geometry_MSB_align_flag();
  }

  auto decGeoFrameNBD = convertBitDepth(inFrame, geoBitDepthNF, geoMSBAlignFlag, 1);
  atlas.geoFrameNF = convertResolution(std::move(decGeoFrameNBD), videoWidthNF, videoHeightNF);
}

void PreRenderer::convertAttributeNominalFormat(const MivBitstream::V3cParameterSet &vps,
                                                MivBitstream::AtlasId atlasId,
                                                MivBitstream::AtlasAccessUnit &atlas,
                                                const Common::FrameList<> &inFrame) {
  const auto videoWidthNF = vps.vps_frame_width(atlasId);
  const auto videoHeightNF = vps.vps_frame_height(atlasId);

  const auto attrCount = Common::downCast<uint8_t>(inFrame.size());

  atlas.attrFrameNF.resize(attrCount);

  for (uint8_t attrIdx = 0; attrIdx < attrCount; ++attrIdx) {
    uint32_t attrBitDepthNF{};
    bool attrMSBAlignFlag{};
    uint8_t attrDim{};

    if (vps.vps_attribute_video_present_flag(atlasId)) {
      const auto &ai = vps.attribute_information(atlasId);
      attrBitDepthNF = ai.ai_attribute_2d_bit_depth_minus1(attrIdx) + 1U;
      attrMSBAlignFlag = ai.ai_attribute_MSB_align_flag(attrIdx);
      attrDim = ai.ai_attribute_dimension_minus1(attrIdx) + 1U;
    } else {
      const auto &pin = vps.packing_information(atlasId);
      attrBitDepthNF = pin.pin_attribute_2d_bit_depth_minus1(attrIdx) + 1U;
      attrMSBAlignFlag = pin.pin_attribute_MSB_align_flag(attrIdx);
      attrDim = pin.pin_attribute_dimension_minus1(attrIdx) + 1U;
    }

    auto decAttrFrameNBD =
        convertBitDepth(inFrame[attrIdx], attrBitDepthNF, attrMSBAlignFlag, attrDim);
    auto decAttrFrameNCF = upsampleChroma(std::move(decAttrFrameNBD));
    atlas.attrFrameNF[attrIdx] =
        convertResolution(std::move(decAttrFrameNCF), videoWidthNF, videoHeightNF);
  }
}

auto PreRenderer::convertBitDepth(Common::Frame<> frame, uint32_t nominalBitDepth,
                                  bool alignmentFlag, uint8_t dimensions) -> Common::Frame<> {
  PRECONDITION(dimensions <= frame.getNumberOfPlanes());
  frame.getPlanes().erase(frame.getPlanes().begin() + dimensions, frame.getPlanes().end());

  const auto bitDepthDifference =
      static_cast<int32_t>(frame.getBitDepth()) - static_cast<int32_t>(nominalBitDepth);

  if (alignmentFlag) {
    for (auto &plane : frame.getPlanes()) {
      for (auto &sample : plane) {
        sample >>= bitDepthDifference;
      }
    }
  } else if (bitDepthDifference > 0) {
    const auto maxValue = Common::maxLevel<uint16_t>(nominalBitDepth);

    for (auto &plane : frame.getPlanes()) {
      for (auto &sample : plane) {
        sample = std::min(sample, maxValue);
      }
    }
  } else if (bitDepthDifference < 0 && alignmentFlag) {
    for (auto &plane : frame.getPlanes()) {
      for (auto &sample : plane) {
        sample <<= (-bitDepthDifference);
      }
    }
  } else {
    // pass-through
  }

  frame.setBitDepth(nominalBitDepth);
  return frame;
}

namespace {
auto convertResolution_(Common::Mat<> inPlane, int32_t videoWidthNF, int32_t videoHeightNF) {
  PRECONDITION(inPlane.width() != 0 && inPlane.height() != 0);

  const auto sizes =
      std::array{static_cast<size_t>(videoHeightNF), static_cast<size_t>(videoWidthNF)};

  if (inPlane.sizes() == sizes) {
    return inPlane; // pass-through
  }

  auto outPlane = Common::Mat<>{sizes};

  for (size_t i = 0; i < outPlane.height(); ++i) {
    const auto n = (i * inPlane.height()) / outPlane.height();

    for (size_t j = 0; j < outPlane.width(); ++j) {
      const auto m = (j * inPlane.width()) / outPlane.width();

      outPlane(i, j) = inPlane(n, m);
    }
  }

  return outPlane;
}
} // namespace

auto PreRenderer::convertResolution(Common::Frame<> inFrame, int32_t videoWidthNF,
                                    int32_t videoHeightNF) -> Common::Frame<> {
  auto outFrame = Common::Frame<>{};
  outFrame.setBitDepth(inFrame.getBitDepth());
  outFrame.getPlanes().resize(inFrame.getPlanes().size());

  std::transform(inFrame.getPlanes().begin(), inFrame.getPlanes().end(),
                 outFrame.getPlanes().begin(),
                 [videoWidthNF, videoHeightNF](Common::Mat<> &inPlane) {
                   return convertResolution_(std::move(inPlane), videoWidthNF, videoHeightNF);
                 });

  return outFrame;
}

auto PreRenderer::upsampleChroma(Common::Frame<> inFrame) -> Common::Frame<> {
  const auto size = inFrame.getSize();
  return convertResolution(std::move(inFrame), size.x(), size.y());
}

auto PreRenderer::thresholdOccupancy(const Common::Frame<> &inFrame, uint8_t threshold)
    -> Common::Frame<bool> {
  auto outFrame = Common::Frame<bool>{inFrame.getSize(), 1, Common::ColorFormat::YUV400};
  const auto inputBitDepth = Common::assertDownCast<int32_t>(inFrame.getBitDepth());
  const auto lossyThreshold = Common::shift(threshold, inputBitDepth - 8);

  PRECONDITION(!inFrame.empty());
  std::transform(inFrame.getPlane(0).cbegin(), inFrame.getPlane(0).cend(),
                 outFrame.getPlane(0).begin(),
                 [lossyThreshold](uint16_t sample) { return lossyThreshold < sample; });

  return outFrame;
}
void PreRenderer::unpackDecodedPackedVideo(const MivBitstream::V3cParameterSet &vps,
                                           MivBitstream::AtlasId atlasId,
                                           MivBitstream::AtlasAccessUnit &atlas) {
  if (atlas.decPckFrame.empty()) {
    return;
  }

  // http://mpegx.int-evry.fr/software/MPEG/PCC/Specs/23090-5/-/issues/486
  const auto decPckFrameNCF = upsampleChroma(atlas.decPckFrame);

  const auto resolutions = calculateUnpackedVideoComponentResolution(vps, atlasId);

  // http://mpegx.int-evry.fr/software/MPEG/PCC/Specs/23090-5/-/issues/493
  const auto decPckBitDepth = decPckFrameNCF.getBitDepth();
  const auto decPckColorFormatNF = decPckFrameNCF.getColorFormat();

  initializeUnpackedVideoComponentFrame(vps.packing_information(atlasId), resolutions,
                                        decPckBitDepth, decPckColorFormatNF, atlas);

  copyDataFromPackedRegionsToUnpackedVideoComponentFrames(vps, atlasId, decPckFrameNCF, atlas);
}

namespace {
// These limitations are acceptable given the superset of all MIV profiles does not include this
// functionality
auto supported(const MivBitstream::PackingInformation &pin, uint8_t regionIdx) {
  switch (pin.pinRegionTypeId(regionIdx)) {
  case MivBitstream::VuhUnitType::V3C_OVD:
    return true;
  case MivBitstream::VuhUnitType::V3C_GVD:
    return pin.pin_region_map_index(regionIdx) == 0 &&
           !pin.pin_region_auxiliary_data_flag(regionIdx);
  case MivBitstream::VuhUnitType::V3C_AVD:
    return pin.pin_region_map_index(regionIdx) == 0 &&
           pin.pin_region_attr_partition_index(regionIdx) == 0 &&
           !pin.pin_region_auxiliary_data_flag(regionIdx);
  default:
    UNREACHABLE;
  }
}
} // namespace

auto PreRenderer::calculateUnpackedVideoComponentResolution(
    const MivBitstream::V3cParameterSet &vps, MivBitstream::AtlasId atlasId)
    -> UnpackedVideoComponentResolutions {
  auto resolutions = UnpackedVideoComponentResolutions{};

  const auto &pin = vps.packing_information(atlasId);

  if (pin.pin_attribute_present_flag()) {
    resolutions.unpckAttrWidth.resize(pin.pin_attribute_count());
    resolutions.unpckAttrHeight.resize(pin.pin_attribute_count());
  }

  for (uint8_t regionIdx = 0; regionIdx <= pin.pin_regions_count_minus1(); ++regionIdx) {
    if (!supported(pin, regionIdx)) {
      continue;
    }

    const auto regionUnpackedOffsetX = pin.pin_region_unpack_top_left_x(regionIdx);
    const auto regionUnpackedOffsetY = pin.pin_region_unpack_top_left_y(regionIdx);
    const auto regionWidth = pin.pin_region_width_minus1(regionIdx) + 1;
    const auto regionHeight = pin.pin_region_height_minus1(regionIdx) + 1;

    int32_t tempWidth{};
    int32_t tempHeight{};

    if (pin.pin_region_rotation_flag(regionIdx)) {
      tempWidth = regionUnpackedOffsetX + regionHeight;
      tempHeight = regionUnpackedOffsetY + regionWidth;
    } else {
      tempWidth = regionUnpackedOffsetX + regionWidth;
      tempHeight = regionUnpackedOffsetY + regionHeight;
    }

    const auto regionTypeId = pin.pinRegionTypeId(regionIdx);

    if (regionTypeId == MivBitstream::VuhUnitType::V3C_OVD) {
      resolutions.unpckOccWidth = std::max(tempWidth, resolutions.unpckOccWidth);
      resolutions.unpckOccHeight = std::max(tempHeight, resolutions.unpckOccHeight);
    } else if (regionTypeId == MivBitstream::VuhUnitType::V3C_GVD) {
      resolutions.unpckGeoWidth = std::max(tempWidth, resolutions.unpckGeoWidth);
      resolutions.unpckGeoHeight = std::max(tempHeight, resolutions.unpckGeoHeight);
    } else if (regionTypeId == MivBitstream::VuhUnitType::V3C_AVD) {
      const auto attrIdx = pin.pin_region_attr_index(regionIdx);

      resolutions.unpckAttrWidth[attrIdx] =
          std::max(tempWidth, resolutions.unpckAttrWidth[attrIdx]);
      resolutions.unpckAttrHeight[attrIdx] =
          std::max(tempHeight, resolutions.unpckAttrHeight[attrIdx]);
    }
  }

  return resolutions;
}

void PreRenderer::initializeUnpackedVideoComponentFrame(
    const MivBitstream::PackingInformation &pin,
    const UnpackedVideoComponentResolutions &resolutions, uint32_t bitDepth,
    Common::ColorFormat colorFormat, MivBitstream::AtlasAccessUnit &atlas) {
  if (pin.pin_occupancy_present_flag()) {
    atlas.unpckOccFrame.create({resolutions.unpckOccWidth, resolutions.unpckOccHeight},
                               atlas.decPckFrame.getBitDepth(), Common::ColorFormat::YUV400);
  }

  if (pin.pin_geometry_present_flag()) {
    atlas.unpckGeoFrame.create({resolutions.unpckGeoWidth, resolutions.unpckGeoHeight},
                               atlas.decPckFrame.getBitDepth(), Common::ColorFormat::YUV400);
  }

  if (pin.pin_attribute_present_flag()) {
    atlas.unpckAttrFrame.resize(pin.pin_attribute_count());

    for (uint8_t attrIdx = 0; attrIdx < pin.pin_attribute_count(); ++attrIdx) {
      const auto unpckAttrWidth = resolutions.unpckAttrWidth[attrIdx];
      const auto unpckAttrHeight = resolutions.unpckAttrHeight[attrIdx];
      const auto unpckAttrDim = pin.pin_attribute_dimension_minus1(attrIdx) + 1;
      const auto attrColorFormat = unpckAttrDim == 1 ? Common::ColorFormat::YUV400 : colorFormat;

      atlas.unpckAttrFrame[attrIdx] =
          Common::Frame<>{{unpckAttrWidth, unpckAttrHeight}, bitDepth, attrColorFormat};
    }
  }
}

void PreRenderer::copyDataFromPackedRegionsToUnpackedVideoComponentFrames(
    const MivBitstream::V3cParameterSet &vps, MivBitstream::AtlasId atlasId,
    const Common::Frame<> &decPckFrameNCF, MivBitstream::AtlasAccessUnit &atlas) {
  const auto &pin = vps.packing_information(atlasId);

  for (uint8_t regionIdx = 0; regionIdx <= pin.pin_regions_count_minus1(); ++regionIdx) {
    if (!supported(pin, regionIdx)) {
      continue;
    }

    const auto regionWidth = pin.pin_region_width_minus1(regionIdx) + 1;
    const auto regionHeight = pin.pin_region_height_minus1(regionIdx) + 1;
    const auto regionPackedOffsetX = pin.pin_region_top_left_x(regionIdx);
    const auto regionPackedOffsetY = pin.pin_region_top_left_y(regionIdx);
    const auto regionRotationFlag = pin.pin_region_rotation_flag(regionIdx);
    const auto regionUnpackedOffsetX = pin.pin_region_unpack_top_left_x(regionIdx);
    const auto regionUnpackedOffsetY = pin.pin_region_unpack_top_left_y(regionIdx);
    const auto regionTypeId = pin.pinRegionTypeId(regionIdx);

    for (int32_t y = 0; y < regionHeight; ++y) {
      for (int32_t x = 0; x < regionWidth; ++x) {
        const auto px = regionPackedOffsetX + x;
        const auto py = regionPackedOffsetY + y;

        int32_t ux{};
        int32_t uy{};

        if (regionRotationFlag) {
          ux = regionUnpackedOffsetX + y;
          uy = regionUnpackedOffsetY + x;
        } else {
          ux = regionUnpackedOffsetX + x;
          uy = regionUnpackedOffsetY + y;
        }

        if (regionTypeId == MivBitstream::VuhUnitType::V3C_OVD) {
          atlas.unpckOccFrame.getPlane(0)(uy, ux) = decPckFrameNCF.getPlane(0)(py, px);
        } else if (regionTypeId == MivBitstream::VuhUnitType::V3C_GVD) {
          atlas.unpckGeoFrame.getPlane(0)(uy, ux) = decPckFrameNCF.getPlane(0)(py, px);
        } else if (regionTypeId == MivBitstream::VuhUnitType::V3C_AVD) {
          const auto attrIdx = pin.pin_region_attr_index(regionIdx);

          for (size_t c = 0; c < atlas.unpckAttrFrame[attrIdx].getNumberOfPlanes(); ++c) {
            atlas.unpckAttrFrame[attrIdx].getPlane(c)(uy, ux) = decPckFrameNCF.getPlane(c)(py, px);
          }
        }
      }
    }
  }
}

void PreRenderer::reconstructOccupancy(const MivBitstream::ViewParamsList &vpl,
                                       MivBitstream::AtlasAccessUnit &atlas) {
  atlas.occFrame.createY({atlas.asps.asps_frame_width(), atlas.asps.asps_frame_height()});
  auto &occPlane = atlas.occFrame.getPlane(0);

  const auto asme_embedded_occupancy_enabled_flag =
      atlas.asps.asps_miv_extension_present_flag() &&
      atlas.asps.asps_miv_extension().asme_embedded_occupancy_enabled_flag();

  for (int32_t i = 0; i < atlas.occFrame.getHeight(); ++i) {
    for (int32_t j = 0; j < atlas.occFrame.getWidth(); ++j) {
      const auto patchIdx = atlas.patchIdx(i, j);
      bool sampleOccFlag{};

      if (patchIdx == Common::unusedPatchIdx) {
        sampleOccFlag = false;
      } else if (!atlas.occFrameNF.empty()) {
        sampleOccFlag = atlas.occFrameNF.getPlane(0)(i, j);
      } else if (asme_embedded_occupancy_enabled_flag) {
        const auto &pp = atlas.patchParamsList[patchIdx];

        const auto occupancyTransform =
            MivBitstream::OccupancyTransform{vpl[pp.atlasPatchProjectionId()], pp};
        sampleOccFlag = occupancyTransform.occupant(atlas.geoFrame.getPlane(0)(i, j));
      } else {
        sampleOccFlag = true;
      }

      occPlane(i, j) = sampleOccFlag;
    }
  }
}

void PreRenderer::filterEntities(MivBitstream::AtlasAccessUnit &atlas) const {
  if (!atlas.asps.asps_miv_extension_present_flag() ||
      0 == atlas.asps.asps_miv_extension().asme_max_entity_id() || !m_entityDecodeRange) {
    return;
  }

  const auto entityDecodeRange = *m_entityDecodeRange;

  for (auto &patchIdx : atlas.blockToPatchMap.getPlane(0)) {
    if (patchIdx != Common::unusedPatchIdx) {
      const auto entityId = atlas.patchParamsList[patchIdx].atlasPatchEntityId();

      if (entityId < entityDecodeRange[0] || entityDecodeRange[1] <= entityId) {
        patchIdx = Common::unusedPatchIdx;
      }
    }
  }
}

void PreRenderer::offsetTexture(const MivBitstream::V3cParameterSet &vps,
                                MivBitstream::AtlasId atlasId,
                                MivBitstream::AtlasAccessUnit &atlas) {
  // http://mpegx.int-evry.fr/software/MPEG/MIV/Specs/23090-12/-/issues/424
  //
  // If there are multiple atttributes of type ATTR_TEXTURE, then only the first one is offset.
  const auto attrIdx = vps.attrIdxOf(atlasId, MivBitstream::AiAttributeTypeId::ATTR_TEXTURE);

  if (!attrIdx) {
    return; // There is no texture attribute
  }

  // Pass-through with copy
  auto &frame = atlas.texFrame;
  frame = atlas.attrFrameNF[*attrIdx];

  if (!atlas.asps.asps_miv_extension_present_flag()) {
    return;
  }

  const auto &asme = atlas.asps.asps_miv_extension();

  if (!asme.asme_patch_attribute_offset_enabled_flag()) {
    return;
  }

  const auto midValue = frame.neutralValue();
  const auto maxValue = frame.maxValue();

  const auto scaledBitCount = asme.asme_patch_attribute_offset_bit_depth_minus1() + 1;
  const auto inputBitDepth = Common::downCast<int32_t>(frame.getBitDepth());
  const auto bitDepthDifference = inputBitDepth - scaledBitCount;

  for (int32_t i = 0; i < frame.getHeight(); ++i) {
    for (int32_t j = 0; j < frame.getWidth(); ++j) {
      const auto patchIdx = atlas.patchIdx(i, j);

      if (patchIdx == Common::unusedPatchIdx) {
        continue;
      }

      const auto &pp = atlas.patchParamsList[patchIdx];

      for (int32_t c = 0; c < 3; c++) {
        // http://mpegx.int-evry.fr/software/MPEG/MIV/RS/TM1/-/issues/596
        const auto atlasPatchAttributeOffset =
            Common::shift(pp.atlasPatchAttributeOffset()[c], bitDepthDifference) - midValue;
        auto &value = frame.getPlane(c)(i, j);
        value = static_cast<uint16_t>(
            std::clamp<int32_t>(value + atlasPatchAttributeOffset, 0, maxValue));
      }
    }
  }
}

void PreRenderer::scaleGeometryVideo(
    const std::optional<MivBitstream::GeometryUpscalingParameters> &gup,
    MivBitstream::AtlasAccessUnit &atlas) const {
  if (!atlas.geoFrameNF.empty()) {
    atlas.geoFrame = m_geometryScaler.scale(atlas, atlas.geoFrameNF, gup);
  }
}
} // namespace TMIV::Decoder
