/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <TMIV/Quantizer/GeometryQuantizer.h>

#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/Common/format.h>
#include <TMIV/MivBitstream/DepthOccupancyTransform.h>
#include <TMIV/MivBitstream/Formatters.h>

namespace TMIV::Quantizer {
void GeometryDistributions::report(const EncoderParams &params) const {
  for (size_t viewIdx = 0; viewIdx < views.size(); ++viewIdx) {
    if (views[viewIdx]) {
      Common::logVerbose(
          "GeometryRanges: view {} with ID {}: min={} median={} max={} (scene unit^-1)", viewIdx,
          params.viewParamsList[viewIdx].viewId, min(views[viewIdx]), median(views[viewIdx]),
          max(views[viewIdx]));
    }
  }
  for (size_t patchIdx = 0; patchIdx < patches.size(); ++patchIdx) {
    if (patches[patchIdx]) {
      Common::logVerbose(
          "GeometryRanges: patch {} with projection ID {}: min={} median={} max={} (scene unit^-1)",
          patchIdx, params.patchParamsList[patchIdx].atlasPatchProjectionId(),
          min(patches[patchIdx]), median(patches[patchIdx]), max(patches[patchIdx]));
    }
  }
}

auto GeometryDistributions::measure(const std::vector<CodableUnit> &buffer)
    -> GeometryDistributions {
  auto result = GeometryDistributions{};

  const auto &params = buffer.front().encoderParams;

  result.views.resize(params.viewParamsList.size());
  result.patches.resize(params.patchParamsList.size());

  for (const auto &unit : buffer) {
    for (const auto &deepFrame : unit.deepFrameList) {
      if (!deepFrame.geometry.empty()) {
        Common::forPixels(deepFrame.texture.getPlane(0).sizes(), [&](size_t i, size_t j) {
          const auto patchIdx = deepFrame.patchIdx.getPlane(0)(i, j);

          if (patchIdx != Common::unusedPatchIdx) {
            const auto &pp = params.patchParamsList[patchIdx];
            const auto viewIdx = params.viewParamsList.indexOf(pp.atlasPatchProjectionId());
            const auto &vp = params.viewParamsList[viewIdx];

            const auto dt = MivBitstream::DepthTransform{vp.dq, Common::sampleBitDepth};
            const auto ot = MivBitstream::OccupancyTransform{vp};

            const auto geoSample = deepFrame.geometry.getPlane(0)(i, j);

            if (ot.occupant(geoSample)) {
              const auto normDisp = dt.expandNormDisp(geoSample);
              result.patches[patchIdx].sample(normDisp);
              result.views[viewIdx].sample(normDisp);
            }
          }
        });
      }
    }
  }
  return result;
}

Configuration::Configuration(const Common::Json &componentNode) {
  PRECONDITION(componentNode.require("haveGeometryVideo").as<bool>());

  dynamicDepthRange = componentNode.require("dynamicDepthRange").as<bool>();
  halveDepthRange = dynamicDepthRange && componentNode.require("halveDepthRange").as<bool>();
  geoBitDepth = componentNode.require("bitDepthGeometryVideo").as<uint32_t>();

  const auto haveOccupancyVideo = componentNode.require("haveOccupancyVideo").as<bool>();
  const auto embeddedOccupancy =
      !haveOccupancyVideo && componentNode.require("embeddedOccupancy").as<bool>();

  if (embeddedOccupancy) {
    depthOccThresholdAsymmetry = componentNode.require("depthOccThresholdAsymmetry").as<double>();
    depthOccThresholdIfSet = componentNode.require("depthOccThresholdIfSet").asVec<double, 2>();
    for (const auto i : {0, 1}) {
      if (!(0.0 < depthOccThresholdIfSet[i])) {
        throw std::runtime_error(
            "The depthOccThresholdIfSet parameter is only used when the encoder "
            "needs to use occupancy. The value 0 is not allowed.");
      }
      if (0.5 <= depthOccThresholdIfSet[i]) {
        throw std::runtime_error(
            "The encoder takes a margin equal to the depth occupancy threshold, so "
            "setting the threshold this high will make it impossible to encode depth. Note that "
            "depthOccThresholdIfSet is normalized on the max. geometry sample value.");
      }
    }
  }

  if (componentNode.require("piecewiseDepthLinearScaling").as<bool>()) {
    // TODO(BK): The existing code did not match with the specification meaning that there is
    // anyhow work to be done. Because of that, and also in view of the MPEG 143 CE 2 schedule, I
    // did not re-implement this proposal. However, I have prepared for it by gathering the full
    // geometry distribution in the `distributions` argument and providing an `icdf` member
    // function.
    NOT_IMPLEMENTED;
  }
}

void GeometryQuantizer::determineDepthRange(const GeometryDistributions &distributions,
                                            EncoderParams &params) const {
  const auto vme_embedded_occupancy_enabled_flag =
      params.vps.vps_miv_extension().vme_embedded_occupancy_enabled_flag();
  const auto casme_depth_low_quality_flag =
      params.casps.casps_miv_extension().casme_depth_low_quality_flag();

  for (size_t viewIdx = 0; viewIdx < params.viewParamsList.size(); ++viewIdx) {
    auto &vp = params.viewParamsList[viewIdx];
    auto far = vp.dq.dq_norm_disp_low();
    auto near = vp.dq.dq_norm_disp_high();
    Common::logVerbose("determineDepthRange: viewIdx={}, far={}, near={} (input range)", viewIdx,
                       far, near);

    if (m_config.dynamicDepthRange && distributions.views[viewIdx]) {
      far = min(distributions.views[viewIdx]);
      near = max(distributions.views[viewIdx]);
      Common::logVerbose("determineDepthRange: viewIdx={}, far:={}, near:={} (dynamic depth range)",
                         viewIdx, far, near);
    }
    if (casme_depth_low_quality_flag && m_config.halveDepthRange) {
      near = 2 * near - far;
      Common::logVerbose("determineDepthRange: viewIdx={}, near:={} (halve depth range)", viewIdx,
                         near);
    }
    if (vme_embedded_occupancy_enabled_flag && vp.hasOccupancy) {
      const auto twoT = 2. * m_config.occThreshold(casme_depth_low_quality_flag);
      Common::logVerbose("determineDepthRange: twoT={}", twoT);

      // Affine map: 2T -> far, 1 -> near
      // Solve for a0: a0 + 2T a1 == far && a0 + 1 a1 == near:
      //   a0 == (far - 2T near) / (1 - 2T)
      vp.dq.dq_norm_disp_low(static_cast<float>((far - twoT * near) / (1. - twoT)));
      vp.dq.dq_norm_disp_high(near);
    } else {
      vp.dq.dq_norm_disp_low(far);
      vp.dq.dq_norm_disp_high(near);
    }
    Common::logVerbose("determineDepthRange: dq_norm_disp_low={}, dq_norm_disp_high={} (result)",
                       vp.dq.dq_norm_disp_low(), vp.dq.dq_norm_disp_high());
  }
  for (auto &pp : params.patchParamsList) {
    const auto atlasIdx = params.vps.indexOf(pp.atlasId());
    const auto bitDepth = params.atlas[atlasIdx].asps.asps_geometry_2d_bit_depth_minus1() + 1U;
    pp.atlasPatch3dOffsetD(0);
    pp.atlasPatch3dRangeD(Common::maxLevel(bitDepth));
  }
}

void GeometryQuantizer::setDepthOccThreshold(EncoderParams &params) const {
  const auto vme_embedded_occupancy_enabled_flag =
      params.vps.vps_miv_extension().vme_embedded_occupancy_enabled_flag();
  const auto casme_depth_low_quality_flag =
      params.casps.casps_miv_extension().casme_depth_low_quality_flag();

  if (vme_embedded_occupancy_enabled_flag) {
    const auto maxValue = std::ldexp(1., static_cast<uint8_t>(m_config.geoBitDepth)) - 1.;
    const auto baseThreshold = m_config.occThreshold(casme_depth_low_quality_flag) * maxValue;
    const auto maxDecoderThreshold = std::floor(2. * baseThreshold);
    PRECONDITION(0 < maxDecoderThreshold);
    const auto asymmetry = casme_depth_low_quality_flag ? 1. : m_config.depthOccThresholdAsymmetry;
    const auto decoderThreshold = std::clamp(asymmetry * baseThreshold, 1., maxDecoderThreshold);
    const auto depthOccThreshold =
        Common::downCast<Common::SampleValue>(std::llround(decoderThreshold));

    for (auto &vp : params.viewParamsList) {
      if (vp.hasOccupancy) {
        vp.dq.dq_depth_occ_threshold_default(depthOccThreshold);
      }
    }
  }
}

auto GeometryQuantizer::transformParams(const GeometryDistributions &distributions,
                                        EncoderParams params) const -> EncoderParams {
  for (size_t viewIdx = 0; viewIdx < params.viewParamsList.size(); ++viewIdx) {
    Common::logVerbose("transformParams: viewIdx={}, dq_norm_disp_low={}, dq_norm_disp_high={}",
                       viewIdx, params.viewParamsList[viewIdx].dq.dq_norm_disp_low(),
                       params.viewParamsList[viewIdx].dq.dq_norm_disp_high());
  }
  determineDepthRange(distributions, params);
  setDepthOccThreshold(params);

  return params;
}

namespace {
void padGeometryFromLeft(const EncoderParams &outParams, Common::DeepFrameList &atlases) noexcept {
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

auto transformOccupancyFrame(const Common::Frame<> &in, unsigned bitDepth) -> Common::Frame<> {
  auto result = Common::Frame<>::lumaOnly(in.getSize(), bitDepth);

  // The occupancy threshold is set to the mid value (512 for 10b), and the non-occupant (low) and
  // occupant (high) levels are set to quarter (256 for 10b) and three quarter (768 for 10b) values
  // respectively.
  const auto low = Common::assertDownCast<uint16_t>(1U << (bitDepth - 2));
  const auto high = Common::assertDownCast<uint16_t>(3U << (bitDepth - 2));

  std::transform(in.getPlane(0).cbegin(), in.getPlane(0).cend(), result.getPlane(0).begin(),
                 [=](auto x) { return 0 < x ? high : low; });
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

auto GeometryQuantizer::transformAtlases(const EncoderParams &inParams,
                                         const EncoderParams &outParams,
                                         const Common::DeepFrameList &inAtlases)
    -> Common::DeepFrameList {
  auto outAtlases = Common::DeepFrameList(inAtlases.size());

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
} // namespace TMIV::Quantizer
