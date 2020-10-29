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

#include <cassert>
#include <iostream>

namespace TMIV::Encoder {
void Encoder::scaleGeometryDynamicRange() {
  assert(m_dynamicDepthRange);
  const auto lowDepthQuality = m_params.vps.vps_miv_extension().vme_depth_low_quality_flag();
  const auto numOfFrames = m_transportViews.size();
  const auto numOfViews = m_transportViews[0].size();

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
        geometry = static_cast<uint16_t>(
            (geometry + 0.5 - minDepthMapValWithinGOP) /
            (static_cast<double>(maxDepthMapValWithinGOP) - minDepthMapValWithinGOP) * 65535.0);
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
}

auto Encoder::completeAccessUnit() -> const MivBitstream::EncoderParams & {
  m_aggregator->completeAccessUnit();
  const auto &aggregatedMask = m_aggregator->getAggregatedMask();

  updateAggregationStatistics(aggregatedMask);

  if (m_dynamicDepthRange) {
    scaleGeometryDynamicRange();
  }

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

void Encoder::calculateAttributeOffset(
    std::vector<std::array<std::array<int64_t, 4>, 3>> patchAttrOffsetValuesFullGOP) {

  int inputBitCount = m_params.vps.attribute_information(m_params.vps.vps_atlas_id(0))
                          .ai_attribute_2d_bit_depth_minus1(0) +
                      1;
  int inputMaxVal = (1 << inputBitCount) - 1;
  int inputMedVal = 1 << (inputBitCount - 1);

  int scaledBitCount = m_attributeOffsetBitCount;
  bool enableMinMin = m_attributeOffsetFlag;

  int bitShift = inputBitCount - scaledBitCount;

  int NOA = m_params.vps.vps_atlas_count_minus1() + 1;
  for (int a = 0; a < NOA; a++) {
    auto &asme = m_params.atlas[a].asme();
    asme.asme_patch_attribute_offset_flag(enableMinMin);
    if (!asme.asme_patch_attribute_offset_flag()) {
      return;
    }
    asme.asme_patch_attribute_offset_bit_count_minus1(scaledBitCount - 1);
  }

  for (int p = 0; p < int(m_params.patchParamsList.size()); p++) {
    for (int c = 0; c < 3; c++) {
      if (patchAttrOffsetValuesFullGOP[p][c][3] > 0) {
        patchAttrOffsetValuesFullGOP[p][c][2] /= patchAttrOffsetValuesFullGOP[p][c][3];
      } else {
        patchAttrOffsetValuesFullGOP[p][c][2] = 0;
      }
      patchAttrOffsetValuesFullGOP[p][c][2] -= inputMedVal; // offset

      if (patchAttrOffsetValuesFullGOP[p][c][0] - patchAttrOffsetValuesFullGOP[p][c][2] < 0) {
        patchAttrOffsetValuesFullGOP[p][c][2] = patchAttrOffsetValuesFullGOP[p][c][0];
      } else if (patchAttrOffsetValuesFullGOP[p][c][1] - patchAttrOffsetValuesFullGOP[p][c][2] >
                 int64_t(inputMaxVal)) {
        patchAttrOffsetValuesFullGOP[p][c][2] = patchAttrOffsetValuesFullGOP[p][c][1] - inputMaxVal;
      }

      patchAttrOffsetValuesFullGOP[p][c][2] += inputMedVal;
      patchAttrOffsetValuesFullGOP[p][c][2] >>= bitShift;
    }

    m_params.patchParamsList[p].atlasPatchAttributeOffset(
        {int(patchAttrOffsetValuesFullGOP[p][0][2]), int(patchAttrOffsetValuesFullGOP[p][1][2]),
         int(patchAttrOffsetValuesFullGOP[p][2][2])});
  }

  // btpm
  std::vector<std::vector<std::vector<int>>> btpm;
  for (int a = 0; a < NOA; a++) {
    auto &currentAtlas = m_videoFrameBuffer[0][a];
    int AH = currentAtlas.texture.getHeight() / m_blockSize;
    int AW = currentAtlas.texture.getWidth() / m_blockSize;
    std::vector<std::vector<int>> tmphw;
    for (int h = 0; h < AH; h++) {
      std::vector<int> tmpw;
      tmpw.reserve(AW);
      for (int w = 0; w < AW; w++) {
        tmpw.push_back(65535);
      }
      tmphw.push_back(tmpw);
    }
    btpm.push_back(tmphw);
  }

  int patchCnt = 0;
  for (const auto &patch : m_params.patchParamsList) {

    size_t atlasId = m_params.vps.indexOf(patch.atlasId);

    auto &currentAtlas = m_videoFrameBuffer[0][atlasId];
    int AH = currentAtlas.texture.getHeight() / m_blockSize;
    int AW = currentAtlas.texture.getWidth() / m_blockSize;

    int w = patch.atlasPatch3dSizeU();
    int h = patch.atlasPatch3dSizeV();
    int xM = patch.atlasPatch3dOffsetU();
    int yM = patch.atlasPatch3dOffsetV();

    for (int dyAligned = 0; dyAligned < h; dyAligned += m_blockSize) {
      for (int dxAligned = 0; dxAligned < w; dxAligned += m_blockSize) {

        for (int dy = dyAligned; dy < dyAligned + m_blockSize; dy++) {
          for (int dx = dxAligned; dx < dxAligned + m_blockSize; dx++) {

            Common::Vec2i pView = {xM + dx, yM + dy};
            Common::Vec2i pAtlas = patch.viewToAtlas(pView);

            int ay = pAtlas.y() / m_blockSize;
            int ax = pAtlas.x() / m_blockSize;

            if (ay < 0 || ax < 0 || ay >= AH || ax >= AW || pAtlas.y() % m_blockSize != 0 ||
                pAtlas.x() % m_blockSize != 0) {
              continue;
            }

            // std::cout << ay << "\t" << ax << "\n";
            btpm[atlasId][ay][ax] = patchCnt;
          }
        }
      }
    }

    patchCnt++;
  }

  // for (int f = 0; f < int(m_videoFrameBuffer.size()); f++) {
  for (auto &videoFrame : m_videoFrameBuffer) {
    for (int a = 0; a < NOA; a++) {
      auto &currentAtlas = videoFrame[a];
      auto &textureAtlasMap = currentAtlas.texture;
      auto &depthAtlasMap = currentAtlas.depth;

      for (int h = 0; h < textureAtlasMap.getHeight(); h++) {
        for (int w = 0; w < textureAtlasMap.getWidth(); w++) {

          int patchid = btpm[a][h / m_blockSize][w / m_blockSize];

          if (patchid != 65535) {

            if (depthAtlasMap.getPlane(0)(h, w) != 0 ||
                m_params.viewParamsList[m_params.patchParamsList[patchid].atlasPatchProjectionId()]
                    .isBasicView) {
              textureAtlasMap.getPlane(0)(h, w) -=
                  ((m_params.patchParamsList[patchid].atlasPatchAttributeOffset()->x()
                    << bitShift) -
                   inputMedVal);
              if (h % 2 == 0 && w % 2 == 0) {
                textureAtlasMap.getPlane(1)(h / 2, w / 2) -=
                    ((m_params.patchParamsList[patchid].atlasPatchAttributeOffset()->y()
                      << bitShift) -
                     inputMedVal);
                textureAtlasMap.getPlane(2)(h / 2, w / 2) -=
                    ((m_params.patchParamsList[patchid].atlasPatchAttributeOffset()->z()
                      << bitShift) -
                     inputMedVal);
              }
            }
          }

        } // w
      }   // h

    } // a
  }   // f
}

void Encoder::constructVideoFrames() {
  int frameId = 0;

  std::vector<std::array<std::array<int64_t, 4>, 3>> patchAttrOffsetValuesFullGOP;

  int inputBitCount = m_params.vps.attribute_information(m_params.vps.vps_atlas_id(0))
                          .ai_attribute_2d_bit_depth_minus1(0) +
                      1;
  int inputMaxVal = (1 << inputBitCount) - 1;

  for (int p = 0; p < int(m_params.patchParamsList.size()); p++) {
    std::array<std::array<int64_t, 4>, 3> tmp{};
    for (int c = 0; c < 3; c++) {
      tmp[c][0] = inputMaxVal;
      tmp[c][1] = 0;
      tmp[c][2] = 0;
      tmp[c][3] = 0;
    }
    patchAttrOffsetValuesFullGOP.push_back(tmp);
  }

  const auto &vps = m_params.vps;
  const auto &vme = vps.vps_miv_extension();

  for (const auto &views : m_transportViews) {
    Common::MVD16Frame atlasList;

    for (size_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      auto &frame = atlasList.emplace_back();
      const auto j = vps.vps_atlas_id(k);
      const auto frameWidth = vps.vps_frame_width(j);
      const auto frameHeight = vps.vps_frame_height(j);

      // TODO(BK): surround with if ( vps_attribute_video_present_flag ) { }
      frame.texture.resize(frameWidth, frameHeight);
      frame.texture.fillNeutral();

      if (m_haveGeometry) {
        frame.depth.resize(frameWidth, frameHeight);
        frame.depth.fillZero();
      }

      if (vps.vps_occupancy_video_present_flag(j)) {
        int occFrameWidth = frameWidth;
        int occFrameHeight = frameHeight;

        if (vme.vme_occupancy_scale_enabled_flag()) {
          const auto &asme = m_params.atlas[k].asme();
          occFrameWidth /= asme.asme_occupancy_scale_factor_x_minus1() + 1;
          occFrameHeight /= asme.asme_occupancy_scale_factor_y_minus1() + 1;
        }
        frame.occupancy.resize(Common::align(occFrameWidth, 2), Common::align(occFrameHeight, 2));
        frame.occupancy.fillZero();
      }
    }

    int patchCnt = 0;
    std::array<std::array<int64_t, 4>, 3> patchAttrOffsetValues1Frame{};

    for (const auto &patch : m_params.patchParamsList) {
      const auto &view = views[patch.atlasPatchProjectionId()];

      if (0 < vme.vme_max_entities_minus1()) {
        Common::MVD16Frame tempViews;
        tempViews.push_back(view);
        const auto &entityViews = entitySeparator(tempViews, *patch.atlasPatchEntityId());
        patchAttrOffsetValues1Frame = writePatchInAtlas(patch, entityViews[0], atlasList, frameId);
      } else {
        patchAttrOffsetValues1Frame = writePatchInAtlas(patch, view, atlasList, frameId);
      }
      for (int c = 0; c < 3; c++) {
        if (patchAttrOffsetValuesFullGOP[patchCnt][c][0] > patchAttrOffsetValues1Frame[c][0]) {
          patchAttrOffsetValuesFullGOP[patchCnt][c][0] = patchAttrOffsetValues1Frame[c][0];
        }
        if (patchAttrOffsetValuesFullGOP[patchCnt][c][1] < patchAttrOffsetValues1Frame[c][1]) {
          patchAttrOffsetValuesFullGOP[patchCnt][c][1] = patchAttrOffsetValues1Frame[c][1];
        }
        patchAttrOffsetValuesFullGOP[patchCnt][c][2] += patchAttrOffsetValues1Frame[c][2];
        patchAttrOffsetValuesFullGOP[patchCnt][c][3] += patchAttrOffsetValues1Frame[c][3];
      }

      patchCnt++;
    }
    m_videoFrameBuffer.push_back(std::move(atlasList));
    ++frameId;
  }
  calculateAttributeOffset(patchAttrOffsetValuesFullGOP);
}

auto Encoder::writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                                const Common::TextureDepth16Frame &view, Common::MVD16Frame &atlas,
                                int frameId) -> std::array<std::array<int64_t, 4>, 3> {
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

  int inputBitCount = m_params.vps.attribute_information(m_params.vps.vps_atlas_id(0))
                          .ai_attribute_2d_bit_depth_minus1(0) +
                      1;
  int inputMaxVal = (1 << inputBitCount) - 1;
  int inputMedVal = 1 << (inputBitCount - 1);

  std::array<int64_t, 3> minValInPatch = {int64_t(inputMaxVal), int64_t(inputMaxVal),
                                          int64_t(inputMaxVal)};
  std::array<int64_t, 3> maxValInPatch = {0, 0, 0};
  std::array<int64_t, 3> sumValInPatch = {0, 0, 0};
  std::array<int64_t, 3> cntValInPatch = {0, 0, 0};

  assert(0 <= xM && xM + w <= inViewParams.ci.ci_projection_plane_width_minus1() + 1);
  assert(0 <= yM && yM + h <= inViewParams.ci.ci_projection_plane_height_minus1() + 1);

  for (int dyAligned = 0; dyAligned < h; dyAligned += m_blockSize) {
    for (int dxAligned = 0; dxAligned < w; dxAligned += m_blockSize) {
      bool isAggregatedMaskBlockNonEmpty = false;
      for (int dy = dyAligned; dy < dyAligned + m_blockSize; dy++) {
        for (int dx = dxAligned; dx < dxAligned + m_blockSize; dx++) {
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

          if (!isAggregatedMaskBlockNonEmpty && m_haveGeometry) {
            depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
            if (m_params.vps.vps_occupancy_video_present_flag(patchParams.atlasId)) {
              occupancyAtlasMap.getPlane(0)(yOcc, xOcc) = 0;
            }
            textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = inputMedVal;
            if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
              textureAtlasMap.getPlane(1)(pAtlas.y() / 2, pAtlas.x() / 2) = inputMedVal;
              textureAtlasMap.getPlane(2)(pAtlas.y() / 2, pAtlas.x() / 2) = inputMedVal;
            }
            continue;
          }

          // Y
          textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) =
              textureViewMap.getPlane(0)(pView.y(), pView.x());

          sumValInPatch[0] += textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
          cntValInPatch[0] += 1;

          if (minValInPatch[0] > textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x())) {
            minValInPatch[0] = textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
          }
          if (maxValInPatch[0] < textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x())) {
            maxValInPatch[0] = textureAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x());
          }

          // UV
          if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
            for (int p = 1; p < 3; ++p) {
              textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2) =
                  textureViewMap.getPlane(p)(pView.y() / 2, pView.x() / 2);

              sumValInPatch[p] += textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
              cntValInPatch[p] += 1;

              if (minValInPatch[p] > textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2)) {
                minValInPatch[p] = textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
              }
              if (maxValInPatch[p] < textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2)) {
                maxValInPatch[p] = textureAtlasMap.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
              }
            }
          }

          // Depth
          if (m_haveGeometry) {
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

  std::array<std::array<int64_t, 4>, 3> ret{};
  for (int c = 0; c < 3; c++) {
    ret[c][0] = minValInPatch[c];
    ret[c][1] = maxValInPatch[c];
    ret[c][2] = sumValInPatch[c];
    ret[c][3] = cntValInPatch[c];
  }

  return ret;
}
} // namespace TMIV::Encoder
