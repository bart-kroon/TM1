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

#include <TMIV/Encoder/Encoder.h>

#include <iostream>

namespace TMIV::Encoder {
void Encoder::scaleGeometryDynamicRange() {
  auto lowDepthQuality = m_params.vps.vps_miv_extension().vme_depth_low_quality_flag();
  auto numOfFrames = m_transportViews.size();
  auto numOfViews = m_transportViews[0].size();

  for (size_t v = 0; v < numOfViews; v++) {
    int minDepthMapValWithinGOP = 65535;
    int maxDepthMapValWithinGOP = 0;

    for (size_t f = 0; f < numOfFrames; f++) {
      for (const auto geometry : m_transportViews[f][v].depth.getPlane(0)) {
        if (geometry < minDepthMapValWithinGOP) {
          minDepthMapValWithinGOP = geometry;
        }
        if (geometry > maxDepthMapValWithinGOP) {
          maxDepthMapValWithinGOP = geometry;
        }
      }
    }

    for (size_t f = 0; f < numOfFrames; f++) {
      for (auto &geometry : m_transportViews[f][v].depth.getPlane(0)) {
        geometry = uint16_t((geometry + 0.5 - minDepthMapValWithinGOP) /
                            (double(maxDepthMapValWithinGOP) - minDepthMapValWithinGOP) * 65535.0);
        if (lowDepthQuality) {
          geometry /= 2;
        }
      }
    }

    const double normDispHighOrig = m_transportParams.viewParamsList[v].dq.dq_norm_disp_high();
    const double normDispLowOrig = m_transportParams.viewParamsList[v].dq.dq_norm_disp_low();

    double normDispHigh =
        maxDepthMapValWithinGOP / 65535.0 * (normDispHighOrig - normDispLowOrig) + normDispLowOrig;
    const double normDispLow =
        minDepthMapValWithinGOP / 65535.0 * (normDispHighOrig - normDispLowOrig) + normDispLowOrig;

    if (lowDepthQuality) {
      normDispHigh = 2 * normDispHigh - normDispLow;
    }

    m_params.viewParamsList[v].dq.dq_norm_disp_high(static_cast<float>(normDispHigh));
    m_params.viewParamsList[v].dq.dq_norm_disp_low(static_cast<float>(normDispLow));
  }
} // namespace TMIV::Encoder

auto Encoder::completeAccessUnit() -> const MivBitstream::EncoderParams & {
  m_aggregator->completeAccessUnit();
  const auto &aggregatedMask = m_aggregator->getAggregatedMask();

  updateAggregationStatistics(aggregatedMask);

  scaleGeometryDynamicRange();

  if (m_params.vme().vme_max_entities_minus1() > 0) {
    m_packer->updateAggregatedEntityMasks(m_aggregatedEntityMask);
  }

  m_params.patchParamsList = m_packer->pack(m_params.atlasSizes(), aggregatedMask,
                                            m_transportParams.viewParamsList, m_blockSize);

  m_params = m_geometryQuantizer->setOccupancyParams(m_params);

  constructVideoFrames();

  const auto &paramsQuantized = m_geometryQuantizer->transformParams(m_params);
  const auto &paramsScaled = m_geometryDownscaler.transformParams(paramsQuantized);
  return paramsScaled;
}

void Encoder::updateAggregationStatistics(const Common::MaskList &aggregatedMask) {
  const auto lumaSamplesPerFrame = std::accumulate(
      aggregatedMask.begin(), aggregatedMask.end(), size_t{}, [](size_t sum, const auto &mask) {
        return sum + 2 * std::count_if(mask.getPlane(0).begin(), mask.getPlane(0).end(),
                                       [](auto x) { return x > 0; });
      });
  std::cout << "Aggregated luma samples per frame is " << (1e-6 * lumaSamplesPerFrame) << "M\n";
  m_maxLumaSamplesPerFrame = std::max(m_maxLumaSamplesPerFrame, lumaSamplesPerFrame);
}

void Encoder::constructVideoFrames() {
  int frame = 0;
  for (const auto &views : m_transportViews) {
    Common::MVD16Frame atlasList;

    for (size_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
      const auto j = m_params.vps.vps_atlas_id(k);
      const auto frameWidth = m_params.vps.vps_frame_width(j);
      const auto frameHeight = m_params.vps.vps_frame_height(j);
      Common::TextureDepth16Frame frame;
      if (m_params.vps.vps_occupancy_video_present_flag(j)) {
        if (m_params.vps.vps_miv_extension().vme_occupancy_scale_enabled_flag()) {
          int codedOccupancyWidth =
              frameWidth / (m_params.atlas[k].asme().asme_occupancy_scale_factor_x_minus1() + 1);
          int codedOccupancyHeight =
              frameHeight / (m_params.atlas[k].asme().asme_occupancy_scale_factor_y_minus1() + 1);
          // make sure coded occupancy maps are divisible by 2 for HM coding functionality
          codedOccupancyWidth = codedOccupancyWidth + codedOccupancyWidth % 2;
          codedOccupancyHeight = codedOccupancyHeight + codedOccupancyHeight % 2;
          frame = {Common::TextureFrame(frameWidth, frameHeight),
                   Common::Depth16Frame(frameWidth, frameHeight),
                   Common::Occupancy10Frame(codedOccupancyWidth, codedOccupancyHeight)};
        } else {
          frame = {Common::TextureFrame(frameWidth, frameHeight),
                   Common::Depth16Frame(frameWidth, frameHeight),
                   Common::Occupancy10Frame(frameWidth, frameHeight)};
        }
      } else {
        frame = {Common::TextureFrame(frameWidth, frameHeight),
                 Common::Depth16Frame(frameWidth, frameHeight)};
      }

      frame.texture.fillNeutral();
      frame.depth.fillZero();
      if (m_params.vps.vps_occupancy_video_present_flag(j)) {
        frame.occupancy.fillZero();
      }
      atlasList.push_back(std::move(frame));
    }

    for (const auto &patch : m_params.patchParamsList) {
      const auto &view = views[patch.atlasPatchProjectionId()];
      if (m_params.vme().vme_max_entities_minus1() > 0) {
        Common::MVD16Frame tempViews;
        tempViews.push_back(view);
        const auto &entityViews = entitySeparator(tempViews, *patch.atlasPatchEntityId());
        writePatchInAtlas(patch, entityViews[0], atlasList, frame);
      } else {
        writePatchInAtlas(patch, view, atlasList, frame);
      }
    }
    m_videoFrameBuffer.push_back(std::move(atlasList));
    frame++;
  }
}

void Encoder::writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                                const Common::TextureDepth16Frame &view, Common::MVD16Frame &atlas,
                                int frameId) {
  const auto k = m_params.vps.indexOf(patchParams.atlasId);
  auto &currentAtlas = atlas[k];

  auto &textureAtlasMap = currentAtlas.texture;
  auto &depthAtlasMap = currentAtlas.depth;
  auto &occupancyAtlasMap = currentAtlas.occupancy;

  const auto &textureViewMap = view.texture;
  const auto &depthViewMap = view.depth;
  const auto w = static_cast<int>(patchParams.atlasPatch3dSizeU());
  const auto h = static_cast<int>(patchParams.atlasPatch3dSizeV());
  const auto xM = static_cast<int>(patchParams.atlasPatch3dOffsetU());
  const auto yM = static_cast<int>(patchParams.atlasPatch3dOffsetV());

  const auto &inViewParams = m_transportParams.viewParamsList[patchParams.atlasPatchProjectionId()];
  const auto &outViewParams = m_params.viewParamsList[patchParams.atlasPatchProjectionId()];

  for (int dyAligned = 0; dyAligned < h; dyAligned += m_blockSize) {
    for (int dxAligned = 0; dxAligned < w; dxAligned += m_blockSize) {
      bool isAggregatedMaskBlockNonEmpty = false;
      for (int dy = dyAligned; dy < dyAligned + m_blockSize; dy++) {
        if (dy + yM >= textureViewMap.getHeight() || dy + yM < 0) {
          continue;
        }
        for (int dx = dxAligned; dx < dxAligned + m_blockSize; dx++) {
          if (dx + xM >= textureViewMap.getWidth() || dx + xM < 0) {
            continue;
          }
          if (m_nonAggregatedMask[patchParams.atlasPatchProjectionId()](dy + yM,
                                                                        dx + xM)[frameId]) {
            isAggregatedMaskBlockNonEmpty = true;
            break;
          }
        }
        if (isAggregatedMaskBlockNonEmpty) {
          break;
        }
      }
      int yOcc = 0;
      int xOcc = 0;
      for (int dy = dyAligned; dy < dyAligned + m_blockSize; dy++) {
        for (int dx = dxAligned; dx < dxAligned + m_blockSize; dx++) {
          Common::Vec2i pView = {xM + dx, yM + dy};
          Common::Vec2i pAtlas = patchParams.viewToAtlas(pView);
          if (m_params.vme().vme_occupancy_scale_enabled_flag()) {
            const auto &asme = m_params.atlas[k].asme();
            yOcc = pAtlas.y() / (asme.asme_occupancy_scale_factor_y_minus1() + 1);
            xOcc = pAtlas.x() / (asme.asme_occupancy_scale_factor_x_minus1() + 1);
          } else {
            yOcc = pAtlas.y();
            xOcc = pAtlas.x();
          }
          if (pView.y() >= textureViewMap.getHeight() || pView.x() >= textureViewMap.getWidth() ||
              pAtlas.y() >= textureAtlasMap.getHeight() ||
              pAtlas.x() >= textureAtlasMap.getWidth() || pView.y() < 0 || pView.x() < 0 ||
              pAtlas.y() < 0 || pAtlas.x() < 0) {
            continue;
          }

          if (!isAggregatedMaskBlockNonEmpty) {
            depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
            if (m_params.vps.vps_occupancy_video_present_flag(patchParams.atlasId)) {
              occupancyAtlasMap.getPlane(0)(yOcc, xOcc) = 0;
            }
            continue;
          }

          // Y
          textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) =
              textureViewMap.getPlane(0)(pView.y(), pView.x());
          // UV
          if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
            for (int p = 1; p < 3; ++p) {
              textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2) =
                  textureViewMap.getPlane(p)(pView.y() / 2, pView.x() / 2);
            }
          }

          // Depth
          auto depth = depthViewMap.getPlane(0)(pView.y(), pView.x());
          if (depth == 0 && !inViewParams.hasOccupancy && outViewParams.hasOccupancy &&
              m_params.vme().vme_max_entities_minus1() == 0) {
            depth = 1; // Avoid marking valid depth as invalid
          }
          depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = depth;
          if (depth > 0 && m_params.vps.vps_occupancy_video_present_flag(patchParams.atlasId)) {
            occupancyAtlasMap.getPlane(0)(yOcc, xOcc) = 1;
          };
        }
      }
    }
  }
}
} // namespace TMIV::Encoder
