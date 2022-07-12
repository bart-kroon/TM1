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

#include "GeometryQuantizer.h"

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

namespace TMIV::Encoder::GeometryQuantizer {
auto transformParams(const EncoderParams &inParams, double depthOccThresholdIfSet,
                     uint32_t bitDepth) -> EncoderParams {
  auto outParams = inParams;

  if (outParams.vps.vps_miv_extension().vme_embedded_occupancy_enabled_flag()) {
    for (auto &x : outParams.viewParamsList) {
      if (x.hasOccupancy) {
        const auto depthOccThreshold = Common::downCast<uint32_t>(
            std::llround(std::ldexp(depthOccThresholdIfSet, Common::downCast<int32_t>(bitDepth))));

        x.dq.dq_depth_occ_threshold_default(depthOccThreshold); // =T
        const auto nearLevel = static_cast<float>(Common::maxLevel(bitDepth));
        const auto farLevel = static_cast<float>(2 * depthOccThreshold);

        // Mapping is [2T, maxValue] --> [old far, near]. What is level 0? (the new far)
        x.dq.dq_norm_disp_low(x.dq.dq_norm_disp_low() +
                              (0.F - farLevel) / (nearLevel - farLevel) *
                                  (x.dq.dq_norm_disp_high() - x.dq.dq_norm_disp_low()));
      }
    }
  }

  return outParams;
}

namespace {
void padGeometryFromLeft(const EncoderParams &outParams, Common::V3cFrameList &atlases) noexcept {
  for (uint8_t i = 0; i <= outParams.vps.vps_atlas_count_minus1(); ++i) {
    const auto j = outParams.vps.vps_atlas_id(i);
    if (outParams.vps.vps_occupancy_video_present_flag(j)) {
      auto &depthAtlasMap = atlases[i].geometry;
      auto depthScale =
          std::array{outParams.atlas[i].asps.asps_frame_height() / depthAtlasMap.getHeight(),
                     outParams.atlas[i].asps.asps_frame_width() / depthAtlasMap.getWidth()};
      const auto &occupancyAtlasMap = atlases[i].occupancy;
      auto occupancyScale =
          std::array{outParams.atlas[i].asps.asps_frame_height() / occupancyAtlasMap.getHeight(),
                     outParams.atlas[i].asps.asps_frame_width() / occupancyAtlasMap.getWidth()};
      for (int32_t y = 0; y < depthAtlasMap.getHeight(); y++) {
        for (int32_t x = 1; x < depthAtlasMap.getWidth(); x++) {
          auto depth = depthAtlasMap.getPlane(0)(y, x);
          const int32_t yOcc = y * depthScale[0] / occupancyScale[0];
          const int32_t xOcc = x * depthScale[1] / occupancyScale[1];
          if (occupancyAtlasMap.getPlane(0)(yOcc, xOcc) == 0 ||
              (depth == 0 &&
               atlases[i].texture.getPlane(0)(y * depthScale[0], x * depthScale[1]) == 512)) {
            depthAtlasMap.getPlane(0)(y, x) = depthAtlasMap.getPlane(0)(y, x - 1);
          }
        }
      }
    }
  }
}

auto transformOccupancyFrame(const Common::Frame<bool> &in, unsigned bitDepth) -> Common::Frame<> {
  auto result = Common::Frame<>::lumaOnly(in.getSize(), bitDepth);

  // The occupancy threshold is set to the mid value (512 for 10b), and the non-occupant (low) and
  // occupant (high) levels are set to quarter (256 for 10b) and three quarter (768 for 10b) values
  // respectively.
  const auto low = Common::assertDownCast<uint16_t>(1U << (bitDepth - 2));
  const auto high = Common::assertDownCast<uint16_t>(3U << (bitDepth - 2));

  std::transform(in.getPlane(0).cbegin(), in.getPlane(0).cend(), result.getPlane(0).begin(),
                 [=](auto x) { return x ? high : low; });
  return result;
}

auto transformAttributeFrame(const Common::Frame<> &inFrame, uint32_t bitDepth) -> Common::Frame<> {
  auto outFrame = Common::Frame<>{inFrame.getSize(), bitDepth, inFrame.getColorFormat()};
  auto outPlane = outFrame.getPlanes().begin();

  const auto bitDepthDifference =
      Common::downCast<int32_t>(bitDepth) - Common::downCast<int32_t>(inFrame.getBitDepth());

  for (const auto &inPlane : inFrame.getPlanes()) {
    std::transform(inPlane.cbegin(), inPlane.cend(), outPlane->begin(),
                   [bitDepthDifference](auto sample) {
                     return Common::assertDownCast<Common::DefaultElement>(
                         Common::shift(sample, bitDepthDifference));
                   });

    ++outPlane;
  }

  return outFrame;
}
} // namespace

auto transformAtlases(const EncoderParams &inParams, const EncoderParams &outParams,
                      const Common::DeepFrameList &inAtlases) -> Common::V3cFrameList {
  auto outAtlases = Common::V3cFrameList(inAtlases.size());

  for (uint8_t k = 0; k <= outParams.vps.vps_atlas_count_minus1(); ++k) {
    const auto atlasId = outParams.vps.vps_atlas_id(k);

    if (outParams.vps.vps_occupancy_video_present_flag(atlasId)) {
      const auto &oi = outParams.vps.occupancy_information(atlasId);
      const auto occBitDepth = oi.oi_occupancy_2d_bit_depth_minus1() + 1U;
      outAtlases[k].occupancy = transformOccupancyFrame(inAtlases[k].occupancy, occBitDepth);
    }

    if (outParams.vps.vps_geometry_video_present_flag(atlasId)) {
      const auto &gi = outParams.vps.geometry_information(atlasId);
      const auto geoBitDepth = gi.gi_geometry_2d_bit_depth_minus1() + 1U;
      outAtlases[k].geometry.createY(inAtlases[k].geometry.getSize(), geoBitDepth);
    }

    if (outParams.vps.vps_attribute_video_present_flag(atlasId)) {
      const auto &ai = outParams.vps.attribute_information(atlasId);

      if (const auto attrIdx =
              outParams.vps.attrIdxOf(atlasId, MivBitstream::AiAttributeTypeId::ATTR_TEXTURE)) {
        const auto texBitDepth = ai.ai_attribute_2d_bit_depth_minus1(*attrIdx) + 1U;
        outAtlases[k].texture = transformAttributeFrame(inAtlases[k].texture, texBitDepth);
      }
    }
  }

  const auto inGeoBitDepth = inAtlases.front().geometry.getBitDepth();
  LIMITATION(std::all_of(inAtlases.cbegin(), inAtlases.cend(), [inGeoBitDepth](const auto &atlas) {
    return atlas.geometry.getBitDepth() == inGeoBitDepth;
  }));
  LIMITATION(inGeoBitDepth == Common::sampleBitDepth);

  for (const auto &patch : outParams.patchParamsList) {
    const auto &inViewParams = inParams.viewParamsList[patch.atlasPatchProjectionId()];
    const auto &outViewParams = outParams.viewParamsList[patch.atlasPatchProjectionId()];
    const auto inOccupancyTransform = MivBitstream::OccupancyTransform{inViewParams};
    const auto inDepthTransform = MivBitstream::DepthTransform{inViewParams.dq, inGeoBitDepth};
    const auto kIn = inParams.vps.indexOf(patch.atlasId());
    const auto kOut = outParams.vps.indexOf(patch.atlasId());
    const auto outBitDepth = outAtlases[kOut].geometry.getBitDepth();
    const auto outDepthTransform =
        MivBitstream::DepthTransform{outViewParams.dq, patch, outBitDepth};

    for (size_t i = 0; i < static_cast<size_t>(patch.atlasPatch2dSizeY()); ++i) {
      for (size_t j = 0; j < static_cast<size_t>(patch.atlasPatch2dSizeX()); ++j) {
        const auto n = i + patch.atlasPatch2dPosY();
        const auto m = j + patch.atlasPatch2dPosX();

        const auto &plane = inAtlases[kIn].geometry.getPlane(0);

        PRECONDITION(n < plane.height() && m < plane.width());

        const auto inLevel = plane(n, m);

        if (inOccupancyTransform.occupant(inLevel)) {
          const auto normDisp = inDepthTransform.expandNormDisp(inLevel);
          const auto outLevel = outDepthTransform.quantizeNormDisp(normDisp, 0);

          outAtlases[kOut].geometry.getPlane(0)(n, m) =
              Common::downCast<Common::DefaultElement>(outLevel);
        }
      }
    }
  }

  if (!outParams.casps.casps_miv_extension().casme_depth_low_quality_flag()) {
    padGeometryFromLeft(outParams, outAtlases);
  }
  return outAtlases;
}
} // namespace TMIV::Encoder::GeometryQuantizer
