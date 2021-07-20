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

#include <TMIV/Encoder/Encoder.h>

#include <iostream>

namespace TMIV::Encoder {
namespace {
constexpr auto textureBitDepth = Common::TextureFrame::getBitDepth();
constexpr auto textureMaxVal = (1 << textureBitDepth) - 1;
constexpr auto textureMedVal = 1 << (textureBitDepth - 1);

struct PatchStats {
  PatchStats() = default;
  PatchStats(int64_t _minVal) : minVal{_minVal} {}
  int64_t minVal{0}, maxVal{0}, sumVal{0}, cntVal{0};
};

void adaptPatchStatsToTexture(std::array<PatchStats, 3> &patchStats,
                              const Common::TextureDepth16Frame &view,
                              Common::TextureDepthFrame<Common::YUV400P16> &atlas,
                              const Common::Vec2i &pView, const Common::Vec2i &pAtlas,
                              Common::Vec3i &colorCorrectionOffset) {
  // Y
  atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x()) =
      view.texture.getPlane(0)(pView.y(), pView.x());

  patchStats[0].sumVal += atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x());
  patchStats[0].cntVal += 1;

  if (patchStats[0].minVal > atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x())) {
    patchStats[0].minVal = atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x());
  }
  if (patchStats[0].maxVal < atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x())) {
    patchStats[0].maxVal = atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x());
  }
  const auto valY =
      int32_t{atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x())} + colorCorrectionOffset.x();
  atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x()) = Common::assertDownCast<uint16_t>(valY);

  // UV
  if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
    for (int p = 1; p < 3; ++p) {
      atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2) =
          view.texture.getPlane(p)(pView.y() / 2, pView.x() / 2);

      Common::at(patchStats, p).sumVal += atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
      Common::at(patchStats, p).cntVal += 1;

      if (Common::at(patchStats, p).minVal >
          atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2)) {
        Common::at(patchStats, p).minVal =
            atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
      }
      if (Common::at(patchStats, p).maxVal <
          atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2)) {
        Common::at(patchStats, p).maxVal =
            atlas.texture.getPlane(p)(pAtlas.y() / 2, pAtlas.x() / 2);
      }
    }
    const auto valU = int32_t{atlas.texture.getPlane(1)(pAtlas.y() / 2, pAtlas.x() / 2)} +
                      colorCorrectionOffset.y();
    const auto valV = int32_t{atlas.texture.getPlane(2)(pAtlas.y() / 2, pAtlas.x() / 2)} +
                      colorCorrectionOffset.z();
    atlas.texture.getPlane(1)(pAtlas.y() / 2, pAtlas.x() / 2) =
        Common::assertDownCast<uint16_t>(valU);
    atlas.texture.getPlane(2)(pAtlas.y() / 2, pAtlas.x() / 2) =
        Common::assertDownCast<uint16_t>(valV);
  }
}
} // namespace

void Encoder::scaleGeometryDynamicRange() {
  PRECONDITION(m_config.dynamicDepthRange);
  const auto lowDepthQuality = m_params.casps.casps_miv_extension().casme_depth_low_quality_flag();
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

    if (maxDepthMapValWithinGOP == minDepthMapValWithinGOP) {
      continue;
    }

    for (size_t f = 0; f < numOfFrames; f++) {
      for (auto &geometry : m_transportViews[f][v].depth.getPlane(0)) {
        geometry = static_cast<uint16_t>(
            (static_cast<double>(geometry) - minDepthMapValWithinGOP) /
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

void Encoder::correctColors() {
  for (auto &patch : m_params.patchParamsList) {
    int sumErrY = 0;
    int sumErrU = 0;
    int sumErrV = 0;

    int cnt = 0;

    Common::Vec3i ccOffset;

    const auto w = patch.atlasPatch3dSizeU();
    const auto h = patch.atlasPatch3dSizeV();
    const auto xM = patch.atlasPatch3dOffsetU();
    const auto yM = patch.atlasPatch3dOffsetV();

    for (size_t frame = 0; frame < m_transportViews.size(); frame++) {
      const auto &view = m_transportViews[frame][patch.atlasPatchProjectionId()];
      const auto &colorCorrectionMap = m_colorCorrectionMaps[frame][patch.atlasPatchProjectionId()];
      const auto &textureViewMap = view.texture;

      for (int32_t y = 0; y < h; y++) {
        for (int32_t x = 0; x < w; x++) {
          const Common::Vec2i pView = {xM + x, yM + y};

          if (pView.y() >= textureViewMap.getHeight() || pView.x() >= textureViewMap.getWidth() ||
              pView.y() < 0 || pView.x() < 0) {
            continue;
          }

          cnt++;

          if (colorCorrectionMap(pView.y(), pView.x()).x() != 0 &&
              m_nonAggregatedMask[patch.atlasPatchProjectionId()](pView.y(), pView.x())[frame]) {
            sumErrY += colorCorrectionMap(pView.y(), pView.x()).x();
            sumErrU += colorCorrectionMap(pView.y(), pView.x()).y();
            sumErrV += colorCorrectionMap(pView.y(), pView.x()).z();
          }
        }
      }
    }

    ccOffset.x() = sumErrY / cnt;
    ccOffset.y() = sumErrU / cnt;
    ccOffset.z() = sumErrV / cnt;

    m_patchColorCorrectionOffset.push_back(ccOffset);
  }
}

auto Encoder::completeAccessUnit() -> const EncoderParams & {
  m_aggregator->completeAccessUnit();
  const auto &aggregatedMask = m_aggregator->getAggregatedMask();

  updateAggregationStatistics(aggregatedMask);

  if (m_config.dynamicDepthRange) {
    scaleGeometryDynamicRange();
  }

  if (0 < m_params.maxEntityId) {
    m_packer->updateAggregatedEntityMasks(m_aggregatedEntityMask);
  }

  auto atlasSizes = Common::SizeVector(m_params.atlas.size());
  std::transform(
      m_params.atlas.cbegin(), m_params.atlas.cend(), atlasSizes.begin(), [](const auto &atlas) {
        return Common::Vec2i{atlas.asps.asps_frame_width(), atlas.asps.asps_frame_height()};
      });

  m_packer->initialize(atlasSizes, m_config.blockSize);
  m_params.patchParamsList = m_packer->pack(atlasSizes, aggregatedMask,
                                            m_transportParams.viewParamsList, m_config.blockSize);

  m_params = m_geometryQuantizer.setOccupancyParams(m_params, m_config.haveGeometry,
                                                    m_config.haveOccupancy);

  if (m_config.colorCorrectionEnabledFlag) {
    correctColors();
  } else {
    for (size_t p = 0; p < m_params.patchParamsList.size(); p++) {
      m_patchColorCorrectionOffset.emplace_back();
    }
  }

  constructVideoFrames();

  const auto &paramsQuantized = m_geometryQuantizer.transformParams(m_params);
  const auto &paramsScaled = m_geometryDownscaler.transformParams(paramsQuantized);

  if (m_config.framePacking) {
    const auto &paramsWithFramePackInfo = m_framePacker.setPackingInformation(paramsScaled);
    return paramsWithFramePackInfo;
  }
  return paramsScaled;
}

void Encoder::updateAggregationStatistics(const Common::MaskList &aggregatedMask) {
  const auto lumaSamplesPerFrame = std::accumulate(
      aggregatedMask.begin(), aggregatedMask.end(), size_t{}, [](size_t sum, const auto &mask) {
        return sum + 2 * std::count_if(mask.getPlane(0).begin(), mask.getPlane(0).end(),
                                       [](auto x) { return x > 0; });
      });
  std::cout << "Aggregated luma samples per frame is "
            << (1e-6 * static_cast<double>(lumaSamplesPerFrame)) << "M\n";
  m_maxLumaSamplesPerFrame = std::max(m_maxLumaSamplesPerFrame, lumaSamplesPerFrame);
}

void Encoder::calculateAttributeOffset(
    std::vector<std::array<std::array<int64_t, 4>, 3>> patchAttrOffsetValuesFullGOP) {
  for (uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    auto &asme = m_params.atlas[k].asps.asps_miv_extension();
    asme.asme_patch_attribute_offset_enabled_flag(m_config.attributeOffsetFlag);
    if (!m_config.attributeOffsetFlag) {
      return;
    }
    asme.asme_patch_attribute_offset_bit_depth_minus1(
        Common::downCast<uint16_t>(m_config.attributeOffsetBitCount - 1));
  }

  const auto bitShift = calculatePatchAttrOffsetValuesFullGOP(patchAttrOffsetValuesFullGOP);

  std::vector<std::vector<std::vector<int>>> btpm = calculateBtpm();

  adaptBtpmToPatchCount(btpm);

  for (auto &videoFrame : m_videoFrameBuffer) {
    for (uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
      auto &atlas = videoFrame[k];
      const auto &asme = m_params.atlas[k].asps.asps_miv_extension();
      auto occScaleX = 1;
      auto occScaleY = 1;
      if (!asme.asme_embedded_occupancy_enabled_flag() &&
          asme.asme_occupancy_scale_enabled_flag()) {
        occScaleX = asme.asme_occupancy_scale_factor_x_minus1() + 1;
        occScaleY = asme.asme_occupancy_scale_factor_y_minus1() + 1;
      }
      for (int y = 0; y < atlas.texture.getHeight(); ++y) {
        for (int x = 0; x < atlas.texture.getWidth(); ++x) {
          const auto patchIndex = btpm[k][y / m_config.blockSize][x / m_config.blockSize];

          if (patchIndex == Common::unusedPatchId ||
              (atlas.depth.getPlane(0)(y, x) == 0 &&
               !m_params
                    .viewParamsList[m_params.patchParamsList[patchIndex].atlasPatchProjectionId()]
                    .isBasicView)) {
            continue;
          }
          if (atlas.occupancy.getPlane(0)(y / occScaleY, x / occScaleX) == 0) {
            continue;
          }
          const auto &pp = m_params.patchParamsList[patchIndex];
          const auto offset = pp.atlasPatchAttributeOffset();
          atlas.texture.getPlane(0)(y, x) -= ((offset.x() << bitShift) - textureMedVal);
          if (y % 2 == 0 && x % 2 == 0) {
            atlas.texture.getPlane(1)(y / 2, x / 2) -= ((offset.y() << bitShift) - textureMedVal);
            atlas.texture.getPlane(2)(y / 2, x / 2) -= ((offset.z() << bitShift) - textureMedVal);
          }
        }
      }
    }
  }
}

void Encoder::adaptBtpmToPatchCount(std::vector<std::vector<std::vector<int>>> &btpm) const {
  int patchCnt = 0;
  for (const auto &patch : m_params.patchParamsList) {
    size_t atlasId = m_params.vps.indexOf(patch.atlasId());

    const auto &currentAtlas = m_videoFrameBuffer[0][atlasId];
    int AH = currentAtlas.texture.getHeight() / m_config.blockSize;
    int AW = currentAtlas.texture.getWidth() / m_config.blockSize;

    int w = patch.atlasPatch3dSizeU();
    int h = patch.atlasPatch3dSizeV();
    int xM = patch.atlasPatch3dOffsetU();
    int yM = patch.atlasPatch3dOffsetV();

    for (int dyAligned = 0; dyAligned < h; dyAligned += m_config.blockSize) {
      for (int dxAligned = 0; dxAligned < w; dxAligned += m_config.blockSize) {
        for (int dy = dyAligned; dy < dyAligned + m_config.blockSize; dy++) {
          for (int dx = dxAligned; dx < dxAligned + m_config.blockSize; dx++) {
            Common::Vec2i pView = {xM + dx, yM + dy};
            Common::Vec2i pAtlas = patch.viewToAtlas(pView);

            int ay = pAtlas.y() / m_config.blockSize;
            int ax = pAtlas.x() / m_config.blockSize;

            if (ay < 0 || ax < 0 || ay >= AH || ax >= AW || pAtlas.y() % m_config.blockSize != 0 ||
                pAtlas.x() % m_config.blockSize != 0) {
              continue;
            }

            btpm[atlasId][ay][ax] = patchCnt;
          }
        }
      }
    }

    patchCnt++;
  }
}

auto Encoder::calculateBtpm() const -> std::vector<std::vector<std::vector<int>>> {
  std::vector<std::vector<std::vector<int>>> btpm;
  for (uint8_t k = 0; k <= m_params.vps.vps_atlas_count_minus1(); ++k) {
    const auto &currentAtlas = m_videoFrameBuffer[0][k];
    int AH = currentAtlas.texture.getHeight() / m_config.blockSize;
    int AW = currentAtlas.texture.getWidth() / m_config.blockSize;
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
  return btpm;
}

auto Encoder::calculatePatchAttrOffsetValuesFullGOP(
    std::vector<std::array<std::array<int64_t, 4>, 3>> &patchAttrOffsetValuesFullGOP) -> int {
  const auto bitShift = textureBitDepth - m_config.attributeOffsetBitCount;

  for (size_t p = 0; p != m_params.patchParamsList.size(); ++p) {
    for (int c = 0; c < 3; c++) {
      if (patchAttrOffsetValuesFullGOP[p][c][3] > 0) {
        patchAttrOffsetValuesFullGOP[p][c][2] /= patchAttrOffsetValuesFullGOP[p][c][3];
      } else {
        patchAttrOffsetValuesFullGOP[p][c][2] = 0;
      }
      patchAttrOffsetValuesFullGOP[p][c][2] -= textureMedVal; // offset

      if (patchAttrOffsetValuesFullGOP[p][c][0] - patchAttrOffsetValuesFullGOP[p][c][2] < 0) {
        patchAttrOffsetValuesFullGOP[p][c][2] = patchAttrOffsetValuesFullGOP[p][c][0];
      } else if (patchAttrOffsetValuesFullGOP[p][c][1] - patchAttrOffsetValuesFullGOP[p][c][2] >
                 static_cast<int64_t>(textureMaxVal)) {
        patchAttrOffsetValuesFullGOP[p][c][2] =
            patchAttrOffsetValuesFullGOP[p][c][1] - textureMaxVal;
      }

      patchAttrOffsetValuesFullGOP[p][c][2] += textureMedVal;
      patchAttrOffsetValuesFullGOP[p][c][2] >>= bitShift;
    }

    m_params.patchParamsList[p].atlasPatchAttributeOffset(
        {static_cast<uint16_t>(patchAttrOffsetValuesFullGOP[p][0][2]),
         static_cast<uint16_t>(patchAttrOffsetValuesFullGOP[p][1][2]),
         static_cast<uint16_t>(patchAttrOffsetValuesFullGOP[p][2][2])});
  }
  return Common::verifyDownCast<int>(bitShift);
}

auto computeOccUnoccupiedLevel(uint8_t occBitDepthMinus1) -> uint16_t {
  return Common::assertDownCast<uint16_t>(1 << (occBitDepthMinus1 - 1));
}

void Encoder::constructVideoFrames() {
  int frameId = 0;

  auto patchAttrOffsetValuesFullGOP = std::vector<std::array<std::array<int64_t, 4>, 3>>{};

  if (m_config.attributeOffsetFlag) {
    for (size_t p = 0; p < m_params.patchParamsList.size(); ++p) {
      std::array<std::array<int64_t, 4>, 3> tmp{};
      for (int c = 0; c < 3; c++) {
        Common::at(tmp, c)[0] = textureMaxVal;
        Common::at(tmp, c)[1] = 0;
        Common::at(tmp, c)[2] = 0;
        Common::at(tmp, c)[3] = 0;
      }
      patchAttrOffsetValuesFullGOP.push_back(tmp);
    }
  }

  const auto &vps = m_params.vps;

  for (const auto &views : m_transportViews) {
    Common::MVD16Frame atlasList;

    for (size_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      auto &frame = atlasList.emplace_back();
      const auto j = vps.vps_atlas_id(k);
      const auto frameWidth = vps.vps_frame_width(j);
      const auto frameHeight = vps.vps_frame_height(j);

      if (m_config.haveTexture) {
        frame.texture.resize(frameWidth, frameHeight);
        frame.texture.fillNeutral();
      }

      if (m_config.haveGeometry) {
        frame.depth.resize(frameWidth, frameHeight);
        frame.depth.fillZero();
      }

      if (vps.vps_occupancy_video_present_flag(j)) {
        int occFrameWidth = frameWidth;
        int occFrameHeight = frameHeight;

        const auto &asme = m_params.atlas[k].asps.asps_miv_extension();

        if (!asme.asme_embedded_occupancy_enabled_flag() &&
            asme.asme_occupancy_scale_enabled_flag()) {
          occFrameWidth /= asme.asme_occupancy_scale_factor_x_minus1() + 1;
          occFrameHeight /= asme.asme_occupancy_scale_factor_y_minus1() + 1;
        }
        frame.occupancy.resize(Common::align(occFrameWidth, 2), Common::align(occFrameHeight, 2));
        const auto unoccupiedLevel = computeOccUnoccupiedLevel(
            vps.occupancy_information(j).oi_occupancy_2d_bit_depth_minus1());
        frame.occupancy.fillValue(unoccupiedLevel);
      } else {
        frame.occupancy.resize(frameWidth, frameHeight);
        frame.occupancy.fillZero();
      }
    }

    int patchCnt = 0;
    auto patchAttrOffsetValues1Frame = std::array<std::array<int64_t, 4>, 3>{};

    for (size_t patchIdx = 0; patchIdx < m_params.patchParamsList.size(); patchIdx++) {
      const auto &patch = m_params.patchParamsList[patchIdx];
      const auto &view = views[patch.atlasPatchProjectionId()];

      const auto k = m_params.vps.indexOf(patch.atlasId());
      if (0 < m_params.atlas[k].asps.asps_miv_extension().asme_max_entity_id()) {
        Common::MVD16Frame tempViews;
        tempViews.push_back(view);
        const auto &entityViews = entitySeparator(tempViews, *patch.atlasPatchEntityId());
        patchAttrOffsetValues1Frame =
            writePatchInAtlas(patch, entityViews[0], atlasList, frameId, patchIdx);
      } else {
        patchAttrOffsetValues1Frame = writePatchInAtlas(patch, view, atlasList, frameId, patchIdx);
      }

      if (m_config.attributeOffsetFlag) {
        for (int c = 0; c < 3; c++) {
          if (patchAttrOffsetValuesFullGOP[patchCnt][c][0] >
              Common::at(patchAttrOffsetValues1Frame, c)[0]) {
            patchAttrOffsetValuesFullGOP[patchCnt][c][0] =
                Common::at(patchAttrOffsetValues1Frame, c)[0];
          }
          if (patchAttrOffsetValuesFullGOP[patchCnt][c][1] <
              Common::at(patchAttrOffsetValues1Frame, c)[1]) {
            patchAttrOffsetValuesFullGOP[patchCnt][c][1] =
                Common::at(patchAttrOffsetValues1Frame, c)[1];
          }
          patchAttrOffsetValuesFullGOP[patchCnt][c][2] +=
              Common::at(patchAttrOffsetValues1Frame, c)[2];
          patchAttrOffsetValuesFullGOP[patchCnt][c][3] +=
              Common::at(patchAttrOffsetValues1Frame, c)[3];
        }
        patchCnt++;
      }
    }
    m_videoFrameBuffer.push_back(std::move(atlasList));
    ++frameId;
  }
  calculateAttributeOffset(patchAttrOffsetValuesFullGOP);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
auto Encoder::writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                                const Common::TextureDepth16Frame &view, Common::MVD16Frame &frame,
                                int frameId, size_t patchIdx)
    -> std::array<std::array<int64_t, 4>, 3> {
  const auto k = m_params.vps.indexOf(patchParams.atlasId());
  auto &atlas = frame[k];

  const auto sizeU = patchParams.atlasPatch3dSizeU();
  const auto sizeV = patchParams.atlasPatch3dSizeV();
  const auto posU = patchParams.atlasPatch3dOffsetU();
  const auto posV = patchParams.atlasPatch3dOffsetV();

  const auto &inViewParams = m_transportParams.viewParamsList[patchParams.atlasPatchProjectionId()];
  const auto &outViewParams = m_params.viewParamsList[patchParams.atlasPatchProjectionId()];

  std::array<PatchStats, 3> patchStats{};
  std::fill(patchStats.begin(), patchStats.end(), PatchStats{textureMaxVal});

  PRECONDITION(0 <= posU && posU + sizeU <= inViewParams.ci.ci_projection_plane_width_minus1() + 1);
  PRECONDITION(0 <= posV &&
               posV + sizeV <= inViewParams.ci.ci_projection_plane_height_minus1() + 1);

  for (int vBlock = 0; vBlock < sizeV; vBlock += m_config.blockSize) {
    for (int uBlock = 0; uBlock < sizeU; uBlock += m_config.blockSize) {
      bool isAggregatedMaskBlockNonEmpty = false;
      for (int v = vBlock; v < vBlock + m_config.blockSize && v < sizeV; v++) {
        for (int u = uBlock; u < uBlock + m_config.blockSize && u < sizeU; u++) {
          const auto viewIdx = patchParams.atlasPatchProjectionId();
          if (m_nonAggregatedMask[viewIdx](v + posV, u + posU)[frameId]) {
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
      for (int v = vBlock; v < vBlock + m_config.blockSize && v < sizeV; ++v) {
        for (int u = uBlock; u < uBlock + m_config.blockSize && u < sizeU; ++u) {
          const auto pView = Common::Vec2i{posU + u, posV + v};
          const auto pAtlas = patchParams.viewToAtlas(pView);

          const auto &asme = m_params.atlas[k].asps.asps_miv_extension();

          if (!asme.asme_embedded_occupancy_enabled_flag() &&
              asme.asme_occupancy_scale_enabled_flag()) {
            yOcc = pAtlas.y() / (asme.asme_occupancy_scale_factor_y_minus1() + 1);
            xOcc = pAtlas.x() / (asme.asme_occupancy_scale_factor_x_minus1() + 1);
          } else {
            yOcc = pAtlas.y();
            xOcc = pAtlas.x();
          }

          if (!isAggregatedMaskBlockNonEmpty && m_config.haveGeometry) {
            adaptAtlas(patchParams, atlas, yOcc, xOcc, pView, pAtlas);
            continue;
          }

          if (m_config.haveTexture) {
            adaptPatchStatsToTexture(patchStats, view, atlas, pView, pAtlas,
                                     m_patchColorCorrectionOffset[patchIdx]);
          }

          // Depth
          if (m_config.haveGeometry) {
            auto depth = view.depth.getPlane(0)(pView.y(), pView.x());
            atlas.occupancy.getPlane(0)(yOcc, xOcc) = 1;
            if (depth == 0 && !inViewParams.hasOccupancy && outViewParams.hasOccupancy &&
                asme.asme_max_entity_id() == 0) {
              depth = 1; // Avoid marking valid depth as invalid
            }
            if (depth == 0 && inViewParams.hasOccupancy) {
              atlas.occupancy.getPlane(0)(yOcc, xOcc) = 0;
            }
            atlas.depth.getPlane(0)(pAtlas.y(), pAtlas.x()) = depth;
            if (depth > 0 && m_params.vps.vps_occupancy_video_present_flag(patchParams.atlasId())) {
              const auto occupiedLevel = Common::assertDownCast<uint16_t>(
                  computeOccUnoccupiedLevel(
                      m_params.vps.occupancy_information(patchParams.atlasId())
                          .oi_occupancy_2d_bit_depth_minus1()) *
                  3);
              atlas.occupancy.getPlane(0)(yOcc, xOcc) = occupiedLevel;
            }
          }
        }
      }
    }
  }

  std::array<std::array<int64_t, 4>, 3> ret{};
  for (int c = 0; c < 3; c++) {
    Common::at(ret, c)[0] = Common::at(patchStats, c).minVal;
    Common::at(ret, c)[1] = Common::at(patchStats, c).maxVal;
    Common::at(ret, c)[2] = Common::at(patchStats, c).sumVal;
    Common::at(ret, c)[3] = Common::at(patchStats, c).cntVal;
  }
  return ret;
}

void Encoder::adaptAtlas(const MivBitstream::PatchParams &patchParams,
                         Common::TextureDepthFrame<Common::YUV400P16> &atlas, int yOcc, int xOcc,
                         const Common::Vec2i &pView, const Common::Vec2i &pAtlas) const {
  atlas.depth.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
  if (m_params.vps.vps_occupancy_video_present_flag(patchParams.atlasId())) {
    const auto unoccupiedLevel =
        computeOccUnoccupiedLevel(m_params.vps.occupancy_information(patchParams.atlasId())
                                      .oi_occupancy_2d_bit_depth_minus1());
    atlas.occupancy.getPlane(0)(yOcc, xOcc) = unoccupiedLevel;
  }
  if (m_config.haveTexture) {
    atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x()) = textureMedVal;
    if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
      atlas.texture.getPlane(1)(pAtlas.y() / 2, pAtlas.x() / 2) = textureMedVal;
      atlas.texture.getPlane(2)(pAtlas.y() / 2, pAtlas.x() / 2) = textureMedVal;
    }
  }
}

} // namespace TMIV::Encoder
