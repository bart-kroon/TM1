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

#include "EncoderImpl.h"
#include "GeometryQuantizer.h"

#include "PiecewiseLinearDepthScaling.h"

#include <TMIV/Common/LoggingStrategyFmt.h>

namespace TMIV::Encoder {
void Encoder::Impl::scaleGeometryDynamicRange() {
  PRECONDITION(m_config.dynamicDepthRange);
  const auto lowDepthQuality = params().casps.casps_miv_extension().casme_depth_low_quality_flag();
  const auto numOfFrames = m_transportViews.size();
  const auto numOfViews = m_transportViews[0].size();

  LIMITATION(std::all_of(m_transportViews.cbegin(), m_transportViews.cend(), [](const auto &frame) {
    return std::all_of(frame.cbegin(), frame.cend(), [](const auto &view) {
      return view.geometry.getBitDepth() == Common::sampleBitDepth;
    });
  }));

  static constexpr int32_t maxValue = Common::maxLevel(Common::sampleBitDepth);
  static constexpr auto maxValD = static_cast<double>(maxValue);

  for (size_t v = 0; v < numOfViews; v++) {
    int32_t minDepthMapValWithinGOP = maxValue;
    int32_t maxDepthMapValWithinGOP = 0;

    for (size_t f = 0; f < numOfFrames; f++) {
      for (const auto geometry : m_transportViews[f][v].geometry.getPlane(0)) {
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

#if ENABLE_M57419
    std::vector<double> mapped_pivot;

    if (m_config.m57419_piecewiseDepthLinearScaling) {
      mapped_pivot = m57419_piecewiseLinearScaleGeometryDynamicRange(
          numOfFrames, v, minDepthMapValWithinGOP, maxDepthMapValWithinGOP, lowDepthQuality);
    } else {
#endif
      for (size_t f = 0; f < numOfFrames; f++) {
        for (auto &geometry : m_transportViews[f][v].geometry.getPlane(0)) {
          geometry = static_cast<Common::DefaultElement>(
              (static_cast<double>(geometry) - minDepthMapValWithinGOP) /
              (static_cast<double>(maxDepthMapValWithinGOP) - minDepthMapValWithinGOP) * maxValD);
          if (lowDepthQuality) {
            geometry /= 2;
          }
        }
      }
#if ENABLE_M57419
    }
#endif

    const double normDispHighOrig = m_transportParams.viewParamsList[v].dq.dq_norm_disp_high();
    const double normDispLowOrig = m_transportParams.viewParamsList[v].dq.dq_norm_disp_low();

    double normDispHigh =
        maxDepthMapValWithinGOP / maxValD * (normDispHighOrig - normDispLowOrig) + normDispLowOrig;
    const double normDispLow =
        minDepthMapValWithinGOP / maxValD * (normDispHighOrig - normDispLowOrig) + normDispLowOrig;

    if (lowDepthQuality && config().halveDepthRange) {
      normDispHigh = 2 * normDispHigh - normDispLow;
    }

    m_params.viewParamsList[v].dq.dq_norm_disp_high(static_cast<float>(normDispHigh));
    m_params.viewParamsList[v].dq.dq_norm_disp_low(static_cast<float>(normDispLow));

#if ENABLE_M57419
    if (m_config.m57419_piecewiseDepthLinearScaling) {
      const auto piece_num = m_config.m57419_intervalNumber;
      m_params.viewParamsList[v].dq.dq_pivot_count_minus1(static_cast<uint8_t>(piece_num - 1));

      m_params.viewParamsList[v].dq.dq_quantization_law(static_cast<uint8_t>(2));

      for (int32_t i = 0; i < piece_num + 1; i++) {
        double normDispMap =
            mapped_pivot[i] / maxValD * (normDispHighOrig - normDispLowOrig) + normDispLowOrig;
        m_params.viewParamsList[v].dq.dq_pivot_norm_disp(i, static_cast<float>(normDispMap));
      }
    }
#endif
  }
}

void Encoder::Impl::setTiles() {
  m_partitionArray.clear();
  m_params.tileParamsLists.clear();
  m_params.tileParamsLists.resize(m_params.atlas.size());
  const auto numPartitionCol = m_config.numberPartitionCol;
  const auto numPartitionRow = m_config.numberPartitionRow;
  const auto singlePartitionPerTileFlag = m_config.singlePartitionPerTileFlag;
  bool singleTileInAtlasFrameFlag = false;
  if ((m_config.toolsetIdc) == (MivBitstream::PtlProfileToolsetIdc::MIV_Extended)) {
    singleTileInAtlasFrameFlag = ((numPartitionCol <= 1) && (numPartitionRow <= 1));
  } else {
    singleTileInAtlasFrameFlag = true;
  }

  if (singleTileInAtlasFrameFlag) // one tile in atlas
  {
    for (size_t atlasIdx = 0; atlasIdx < m_params.atlas.size(); ++atlasIdx) {
      MivBitstream::TilePartition t;
      t.partitionHeight(m_params.atlas[atlasIdx].asps.asps_frame_height());
      t.partitionWidth(m_params.atlas[atlasIdx].asps.asps_frame_width());
      t.partitionPosX(0);
      t.partitionPosY(0);
      m_params.tileParamsLists[atlasIdx].emplace_back(t);

      // setAtlasFrameTileInformationSnytax
      auto afti = TMIV::MivBitstream::AtlasFrameTileInformation{};
      afti.afti_single_tile_in_atlas_frame_flag(true).afti_num_tiles_in_atlas_frame_minus1(0);
      m_params.atlas[atlasIdx].afps.atlas_frame_tile_information(afti);
    }
  } else {
    bool uniformPartitionSpacingFlag = setPartition();

    if (singlePartitionPerTileFlag) {
      for (size_t atlasIdx = 0; atlasIdx < m_params.atlas.size(); ++atlasIdx) {
        std::vector<MivBitstream::TilePartition> tileList;
        for (size_t i = 0; i < m_partitionArray[1][atlasIdx].size(); ++i) {
          for (size_t j = 0; j < m_partitionArray[0][atlasIdx].size(); ++j) {
            MivBitstream::TilePartition t;
            t.partitionWidth(m_partitionArray[0][atlasIdx][j]);
            t.partitionHeight(m_partitionArray[1][atlasIdx][i]);
            t.partitionPosX(m_partitionArray[2][atlasIdx][j]);
            t.partitionPosY(m_partitionArray[3][atlasIdx][i]);
            tileList.emplace_back(t);
          }
        }
        m_params.tileParamsLists[atlasIdx] = tileList;
      }
    } else {
      // TODO
    }

    setAtlasFrameTileInformation(uniformPartitionSpacingFlag, singlePartitionPerTileFlag);
  }

  for (size_t atlasIdx = 0; atlasIdx < m_params.tileParamsLists.size(); ++atlasIdx) {
    setAtlasTileHeaderSnytax(atlasIdx);
  }
}

auto Encoder::Impl::setPartition() -> bool {
  const auto partitionWidth = m_config.partitionWidth;
  const auto partitionHeight = m_config.partitionHeight;
  const auto numPartitionCol = m_config.numberPartitionCol;
  const auto numPartitionRow = m_config.numberPartitionRow;
  bool uniformPartitionSpacingFlagWidth = true;
  bool uniformPartitionSpacingFlagHeight = true;

  int32_t widthSum = 0;
  int32_t heightSum = 0;

  for (const auto &w : partitionWidth) {
    widthSum = widthSum + w;
  }
  for (const auto &h : partitionHeight) {
    heightSum = heightSum + h;
  }

  if (widthSum != m_params.atlas.front().asps.asps_frame_width()) {
    TMIV::Common::logWarning("the sum of tile width={}, the atlas width={}", widthSum,
                             m_params.atlas.front().asps.asps_frame_width());
    throw std::runtime_error("Atlas width should be equal to sum of tile width");
  }
  if (heightSum != m_params.atlas.front().asps.asps_frame_height()) {
    TMIV::Common::logWarning("the sum of tile height={}, the atlas height={}", heightSum,
                             m_params.atlas.front().asps.asps_frame_height());
    throw std::runtime_error("Atlas height should be equal to sum of tile height");
  }

  for (size_t i = 0; i < partitionWidth.size() - 1; ++i) {
    if (partitionWidth[0] != partitionWidth[i] || partitionWidth[0] < partitionWidth.back()) {
      uniformPartitionSpacingFlagWidth = false;
      break;
    }
  }
  for (size_t j = 0; j < partitionHeight.size() - 1; ++j) {
    if (partitionHeight[0] != partitionHeight[j] || partitionHeight[0] < partitionHeight.back()) {
      uniformPartitionSpacingFlagHeight = false;
      break;
    }
  }
  std::vector<std::vector<int32_t>> partitionWidthList;
  std::vector<std::vector<int32_t>> partitionHeightList;
  std::vector<std::vector<int32_t>> partitionPosXList;
  std::vector<std::vector<int32_t>> partitionPosYList;
  for (size_t atlasIdx = 0; atlasIdx < m_params.atlas.size(); ++atlasIdx) {
    std::vector<int32_t> widthList;
    std::vector<int32_t> heightList;
    std::vector<int32_t> posXList;
    std::vector<int32_t> posYList;
    int32_t widthTemp = 0;
    int32_t heightTemp = 0;
    for (int32_t j = 0; j < numPartitionCol; ++j) {
      widthList.emplace_back(partitionWidth[j]);
      posXList.emplace_back(widthTemp);
      widthTemp += partitionWidth[j];
    }
    for (int32_t i = 0; i < numPartitionRow; ++i) {
      heightList.emplace_back(partitionHeight[i]);
      posYList.emplace_back(heightTemp);
      heightTemp += partitionHeight[i];
    }
    partitionHeightList.emplace_back(heightList);
    partitionPosXList.emplace_back(posXList);
    partitionPosYList.emplace_back(posYList);
    partitionWidthList.emplace_back(widthList);
  }

  m_partitionArray.emplace_back(partitionWidthList);
  m_partitionArray.emplace_back(partitionHeightList);
  m_partitionArray.emplace_back(partitionPosXList);
  m_partitionArray.emplace_back(partitionPosYList);

  return (uniformPartitionSpacingFlagWidth && uniformPartitionSpacingFlagHeight);
}

void Encoder::Impl::updateTile() {
  size_t patchNum = m_params.patchParamsList.size();
  size_t tilePatchNum = 0;
  for (const auto &patch : m_params.patchParamsList) {
    size_t atlasID = patch.atlasId().asInt();
    auto patchPosX = patch.atlasPatch2dPosX();
    auto patchPosY = patch.atlasPatch2dPosY();
    auto patchSizeX = patch.atlasPatch2dSizeX();
    auto patchSizeY = patch.atlasPatch2dSizeY();
    for (auto &tile : m_params.tileParamsLists[atlasID]) {
      auto tilePosX = tile.partitionPosX();
      auto tilePosY = tile.partitionPosY();
      auto tileSizeX = tile.partitionWidth();
      auto tileSizeY = tile.partitionHeight();

      if (patchPosX >= tilePosX && patchPosY >= tilePosY &&
          (patchPosX + patchSizeX) <= (tilePosX + tileSizeX) &&
          (patchPosY + patchSizeY) <= (tilePosY + tileSizeY)) {
        tile.addPatchToTile(patch);
        ++tilePatchNum;
        break;
      }
    }
  }

  POSTCONDITION(tilePatchNum == patchNum);
}

void Encoder::Impl::setAtlasFrameTileInformation(bool uniformPartitionSpacingFlag,
                                                 bool partitionPerTileFlag) {
  for (size_t atlasIdx = 0; atlasIdx < m_params.atlas.size(); ++atlasIdx) {
    auto afti = TMIV::MivBitstream::AtlasFrameTileInformation{};
    afti.afti_single_tile_in_atlas_frame_flag(false);
    afti.afti_uniform_partition_spacing_flag(uniformPartitionSpacingFlag);
    afti.afti_single_partition_per_tile_flag(partitionPerTileFlag);
    if (uniformPartitionSpacingFlag) {
      int32_t partitionColsWidthMinus1 = m_config.partitionWidth[0] / 64 - 1;
      VERIFY(m_config.partitionWidth[0] % 64 == 0);

      int32_t partitionRowsHeightMinus1 = m_config.partitionHeight[0] / 64 - 1;
      VERIFY(m_config.partitionHeight[0] % 64 == 0);

      afti.afti_partition_cols_width_minus1(partitionColsWidthMinus1)
          .afti_partition_rows_height_minus1(partitionRowsHeightMinus1);
    } else {
      const int32_t numPartitionColumnsMinus1 = m_config.numberPartitionCol - 1;
      const int32_t numPartitionRowsMinus1 = m_config.numberPartitionRow - 1;
      afti.afti_num_partition_columns_minus1(numPartitionColumnsMinus1)
          .afti_num_partition_rows_minus1(numPartitionRowsMinus1);

      auto partitionColumnWidthMinus1 = std::vector<int32_t>(numPartitionColumnsMinus1, 0);
      auto partitionRowHeightMinus1 = std::vector<int32_t>(numPartitionRowsMinus1, 0);

      for (int32_t j = 0; j < numPartitionColumnsMinus1; ++j) {
        partitionColumnWidthMinus1[j] = m_partitionArray[0][atlasIdx][j] / 64 - 1;
        if (m_partitionArray[0][atlasIdx][j] % 64 != 0) {
          TMIV::Common::logWarning("the width of the tile partition={}",
                                   m_partitionArray[0][atlasIdx][j]);
          throw std::runtime_error("the width of the tile partition should be in units of 64 "
                                   "samples accoding to 23090-5 clause 8.3.6.2.2");
        }
      }
      for (int32_t j = 0; j < numPartitionRowsMinus1; ++j) {
        partitionRowHeightMinus1[j] = m_partitionArray[1][atlasIdx][j] / 64 - 1;
        if (m_partitionArray[1][atlasIdx][j] % 64 != 0) {
          TMIV::Common::logWarning("the height of the tile partition={}",
                                   m_partitionArray[1][atlasIdx][j]);
          throw std::runtime_error("the height of the tile partition should be in units of 64 "
                                   "samples accoding to 23090-5 clause 8.3.6.2.2");
        }
      }

      afti.afti_partition_column_width_minus1(partitionColumnWidthMinus1)
          .afti_partition_row_height_minus1(partitionRowHeightMinus1);
    }
    if (!partitionPerTileFlag) {
      // TODO
    }

    m_params.atlas[atlasIdx].afps.atlas_frame_tile_information(afti);
  }
}

void Encoder::Impl::setAtlasTileHeaderSnytax(size_t atlasIdx) {
  uint8_t psqx = 0;
  uint8_t psqy = 0;
  if (m_params.atlas[atlasIdx].asps.asps_patch_size_quantizer_present_flag()) {
    psqx = m_params.atlas[atlasIdx].athList[0].ath_patch_size_x_info_quantizer();
    psqy = m_params.atlas[atlasIdx].athList[0].ath_patch_size_y_info_quantizer();
  }
  m_params.atlas[atlasIdx].athList.clear();
  auto &atlas = m_params.atlas[atlasIdx];

  for (uint8_t tileIdx = 0;
       tileIdx < Common::downCast<uint8_t>(m_params.tileParamsLists[atlasIdx].size()); ++tileIdx) {
    auto ath = MivBitstream::AtlasTileHeader{};
    ath.ath_type(MivBitstream::AthType::I_TILE)
        .ath_ref_atlas_frame_list_asps_flag(true)
        .ath_pos_min_d_quantizer(atlas.asps.asps_geometry_3d_bit_depth_minus1() + 1);
    if (atlas.asps.asps_patch_size_quantizer_present_flag()) {
      ath.ath_patch_size_x_info_quantizer(psqx);
      ath.ath_patch_size_y_info_quantizer(psqy);
    }
    ath.ath_id(tileIdx);

    atlas.athList.push_back(ath);
  }
}

auto Encoder::Impl::completeAccessUnit() -> const EncoderParams & {
  Common::logVerbose("completeAccessUnit: FOC is {}.", m_params.foc);

  m_aggregator->completeAccessUnit();
  const auto &aggregatedMask = m_aggregator->getAggregatedMask();
  updateAggregationStatistics(aggregatedMask);

  if (m_config.dynamicDepthRange) {
    scaleGeometryDynamicRange();
  }

  if (0 < m_config.maxEntityId) {
    m_packer->updateAggregatedEntityMasks(m_aggregatedEntityMask);
  }

  setTiles();
  m_packer->initialize(m_params.tileParamsLists);

  auto atlasSizes = std::vector<Common::SizeVector>(params().atlas.size());
  for (size_t atlasIdx = 0; atlasIdx < params().atlas.size(); ++atlasIdx) {
    auto &tileSizes = atlasSizes[atlasIdx];
    tileSizes = Common::SizeVector(m_params.tileParamsLists[atlasIdx].size());
    for (size_t tileIdx = 0; tileIdx < m_params.tileParamsLists[atlasIdx].size(); ++tileIdx) {
      tileSizes[tileIdx] =
          Common::Vec2i{m_params.tileParamsLists[atlasIdx][tileIdx].partitionWidth(),
                        m_params.tileParamsLists[atlasIdx][tileIdx].partitionHeight()};
    }
  }

  m_packer->initialize(atlasSizes, m_blockSize);
  m_params.patchParamsList =
      m_packer->pack(atlasSizes, aggregatedMask, m_transportParams.viewParamsList, m_blockSize);

  // NOTE(BK): There is no encoder support for per-patch D range
  for (auto &pp : m_params.patchParamsList) {
    const auto atlasIdx = params().vps.indexOf(pp.atlasId());
    const auto bitDepth = params().atlas[atlasIdx].asps.asps_geometry_2d_bit_depth_minus1() + 1U;
    pp.atlasPatch3dOffsetD(0);
    pp.atlasPatch3dRangeD(Common::maxLevel(bitDepth));
  }

  for (size_t p = 0; p < params().patchParamsList.size(); p++) {
    m_patchColorCorrectionOffset.emplace_back();
  }

  constructVideoFrames();

  updateTile();
  bool occth_type = params().casps.casps_miv_extension().casme_depth_low_quality_flag();
  double depthOccThresholdAsymmetry = m_config.depthOccThresholdAsymmetry;
  m_paramsQuantized =
      GeometryQuantizer::transformParams(params(), m_config.occThreshold(occth_type),
                                         m_config.geoBitDepth, depthOccThresholdAsymmetry);
  m_params.foc += Common::downCast<int32_t>(m_videoFrameBuffer.size());
  m_params.foc %= m_config.intraPeriod;
  Common::logInfo("completeAccessUnit: Added {} frames. Updating FOC to {}.",
                  m_videoFrameBuffer.size(), m_params.foc);

  if (m_config.framePacking) {
    return m_framePacker.setPackingInformation(m_paramsQuantized, m_config.geometryPacking);
  }

  return m_paramsQuantized;
}

void Encoder::Impl::updateAggregationStatistics(const Common::FrameList<uint8_t> &aggregatedMask) {
  const auto lumaSamplesPerFrame = std::accumulate(
      aggregatedMask.begin(), aggregatedMask.end(), size_t{}, [](size_t sum, const auto &mask) {
        return sum + 2 * std::count_if(mask.getPlane(0).begin(), mask.getPlane(0).end(),
                                       [](auto x) { return x > 0; });
      });
  Common::logInfo("Aggregated luma samples per frame is {}M",
                  1e-6 * static_cast<double>(lumaSamplesPerFrame));
  m_maxLumaSamplesPerFrame = std::max(m_maxLumaSamplesPerFrame, lumaSamplesPerFrame);
}

void Encoder::Impl::applyPatchTextureOffset() {
  std::vector<std::vector<std::vector<int32_t>>> btpm = calculateBtpm();

  adaptBtpmToPatchCount(btpm);

  const auto inputBitDepth = m_videoFrameBuffer.front().front().texture.getBitDepth();
  const auto bitShift =
      static_cast<int32_t>(inputBitDepth) - static_cast<int32_t>(m_config.texBitDepth);

  for (auto &videoFrame : m_videoFrameBuffer) {
    for (uint8_t k = 0; k <= params().vps.vps_atlas_count_minus1(); ++k) {
      auto &atlas = videoFrame[k];
      const auto &asme = params().atlas[k].asps.asps_miv_extension();
      auto occScaleX = 1;
      auto occScaleY = 1;

      if (!asme.asme_embedded_occupancy_enabled_flag() &&
          asme.asme_occupancy_scale_enabled_flag()) {
        occScaleX = asme.asme_occupancy_scale_factor_x_minus1() + 1;
        occScaleY = asme.asme_occupancy_scale_factor_y_minus1() + 1;
      }

      for (int32_t y = 0; y < atlas.texture.getHeight(); ++y) {
        for (int32_t x = 0; x < atlas.texture.getWidth(); ++x) {
          const auto patchIdx = btpm[k][y / m_blockSize][x / m_blockSize];

          if (patchIdx == Common::unusedPatchIdx ||
              (atlas.geometry.getPlane(0)(y, x) == 0 &&
               !params()
                    .viewParamsList[params().patchParamsList[patchIdx].atlasPatchProjectionId()]
                    .isBasicView)) {
            continue;
          }
          if (!atlas.occupancy.getPlane(0)(y / occScaleY, x / occScaleX)) {
            continue;
          }
          const auto &pp = params().patchParamsList[patchIdx];

          const auto applyOffset = [&](uint8_t c, int32_t i, int32_t j) {
            auto &sample = atlas.texture.getPlane(c)(i, j);
            sample = Common::assertDownCast<Common::DefaultElement>(
                sample - Common::shift(pp.atlasPatchTextureOffset(c), bitShift));
          };

          applyOffset(0, y, x);

          if (y % 2 == 0 && x % 2 == 0) {
            applyOffset(1, y / 2, x / 2);
            applyOffset(2, y / 2, x / 2);
          }
        }
      }
    }
  }
}

void Encoder::Impl::adaptBtpmToPatchCount(
    std::vector<std::vector<std::vector<int32_t>>> &btpm) const {
  int32_t patchCnt = 0;
  for (const auto &patch : params().patchParamsList) {
    size_t atlasId = params().vps.indexOf(patch.atlasId());

    const auto &currentAtlas = m_videoFrameBuffer[0][atlasId];
    int32_t AH = currentAtlas.texture.getHeight() / m_blockSize;
    int32_t AW = currentAtlas.texture.getWidth() / m_blockSize;

    int32_t w = patch.atlasPatch3dSizeU();
    int32_t h = patch.atlasPatch3dSizeV();
    int32_t xM = patch.atlasPatch3dOffsetU();
    int32_t yM = patch.atlasPatch3dOffsetV();

    for (int32_t dyAligned = 0; dyAligned < h; dyAligned += m_blockSize) {
      for (int32_t dxAligned = 0; dxAligned < w; dxAligned += m_blockSize) {
        for (int32_t dy = dyAligned; dy < dyAligned + m_blockSize; dy++) {
          for (int32_t dx = dxAligned; dx < dxAligned + m_blockSize; dx++) {
            Common::Vec2i pView = {xM + dx, yM + dy};
            Common::Vec2i pAtlas = patch.viewToAtlas(pView);

            int32_t ay = pAtlas.y() / m_blockSize;
            int32_t ax = pAtlas.x() / m_blockSize;

            if (ay < 0 || ax < 0 || ay >= AH || ax >= AW || pAtlas.y() % m_blockSize != 0 ||
                pAtlas.x() % m_blockSize != 0) {
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

auto Encoder::Impl::calculateBtpm() const -> std::vector<std::vector<std::vector<int32_t>>> {
  std::vector<std::vector<std::vector<int32_t>>> btpm;
  for (uint8_t k = 0; k <= params().vps.vps_atlas_count_minus1(); ++k) {
    const auto &currentAtlas = m_videoFrameBuffer[0][k];
    int32_t AH = currentAtlas.texture.getHeight() / m_blockSize;
    int32_t AW = currentAtlas.texture.getWidth() / m_blockSize;
    std::vector<std::vector<int32_t>> tmphw;
    for (int32_t h = 0; h < AH; h++) {
      std::vector<int32_t> tmpw;
      tmpw.reserve(AW);
      for (int32_t w = 0; w < AW; w++) {
        tmpw.push_back(Common::unusedPatchIdx);
      }
      tmphw.push_back(tmpw);
    }
    btpm.push_back(tmphw);
  }
  return btpm;
}

void Encoder::Impl::encodePatchTextureOffset(const PatchTextureStats &stats) {
  const auto inputBitDepth = m_videoFrameBuffer.front().front().texture.getBitDepth();

  const auto muddle = [this, inputBitDepth]() {
    // Take into account three different bit depths
    const auto outputBitDepth = m_config.texBitDepth;
    const auto offsetBitDepth = m_config.textureOffsetBitCount;
    const auto minBitDepth = std::min({inputBitDepth, outputBitDepth, offsetBitDepth});

    // Implement round-towards-zero behaviour
    static_assert((-13 >> 2) == -4); // bad
    static_assert((-13 / 4) == -3);  // good
    const auto divisor = int64_t{1} << (inputBitDepth - minBitDepth);
    const auto scaler = int64_t{1} << (outputBitDepth - minBitDepth);

    return [=](int64_t x) { return Common::downCast<int32_t>((x / divisor) * scaler); };
  }();

  const int64_t m = Common::medLevel(inputBitDepth);
  const int64_t L = Common::maxLevel(inputBitDepth);

  // Constrained optimization to find the atlas patch texture offset o, at input bit depth:
  //
  // input        :       0 <= s.min()     <= s.mean()     <= s.max()     <= L
  // output       :       0 <= s.min() - o <= s.mean() - o <= s.max() - o <= L
  // target       : s.mean(s) - o == m            =>            o == mean(s) - m
  // constraint 1 :             0 <= s.min() - o  =>            o <= s.min()
  // constraint 2 :   s.max() - o <= L            =>  s.max() - L <= o

  for (size_t p = 0; p != params().patchParamsList.size(); ++p) {
    for (uint8_t c = 0; c < 3; ++c) {
      const auto &s = stats[p][c];
      m_params.patchParamsList[p].atlasPatchTextureOffset(
          c, muddle(std::clamp(s.floorMean() - m, s.max() - L, s.min())));
    }
  }
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity,readability-function-size)
void Encoder::Impl::constructVideoFrames() {
  int32_t frameIdx = 0;

  const auto &vps = params().vps;

  auto patchTextureStats = PatchTextureStats(params().patchParamsList.size());

  for (const auto &views : m_transportViews) {
    Common::DeepFrameList atlasList;

    for (size_t k = 0; k <= vps.vps_atlas_count_minus1(); ++k) {
      auto &frame = atlasList.emplace_back();
      const auto j = vps.vps_atlas_id(k);
      const auto frameWidth = vps.vps_frame_width(j);
      const auto frameHeight = vps.vps_frame_height(j);

      if (m_config.haveTexture) {
        const auto texBitDepth = views.front().texture.getBitDepth();

        LIMITATION(std::all_of(views.cbegin(), views.cend(),
                               [texBitDepth](const Common::DeepFrame &frame) {
                                 return frame.texture.getBitDepth() == texBitDepth;
                               }));

        frame.texture.createYuv420({frameWidth, frameHeight}, texBitDepth);
        frame.texture.fillNeutral();
      }

      if (m_config.haveGeometry) {
        frame.geometry.createY({frameWidth, frameHeight});
        frame.geometry.fillZero();
      }

      if (vps.vps_occupancy_video_present_flag(j)) {
        int32_t occFrameWidth = frameWidth;
        int32_t occFrameHeight = frameHeight;

        const auto &asme = params().atlas[k].asps.asps_miv_extension();

        if (!asme.asme_embedded_occupancy_enabled_flag() &&
            asme.asme_occupancy_scale_enabled_flag()) {
          occFrameWidth /= asme.asme_occupancy_scale_factor_x_minus1() + 1;
          occFrameHeight /= asme.asme_occupancy_scale_factor_y_minus1() + 1;
        }
        frame.occupancy.createY({occFrameWidth, occFrameHeight});
      } else {
        frame.occupancy.createY({frameWidth, frameHeight});
      }

      frame.occupancy.fillZero();
    }

    for (size_t p = 0; p < params().patchParamsList.size(); p++) {
      const auto &patch = params().patchParamsList[p];
      const auto viewIdx = params().viewParamsList.indexOf(patch.atlasPatchProjectionId());
      const auto &view = views[viewIdx];
      const auto k = params().vps.indexOf(patch.atlasId());

      if (0 < params().atlas[k].asps.asps_miv_extension().asme_max_entity_id()) {
        Common::DeepFrameList tempViews;
        tempViews.push_back(view);
        const auto &entityViews = entitySeparator(tempViews, patch.atlasPatchEntityId());
        patchTextureStats[p] += writePatchInAtlas(patch, entityViews[0], atlasList, frameIdx, p);
      } else {
        patchTextureStats[p] += writePatchInAtlas(patch, view, atlasList, frameIdx, p);
      }
    }
    m_videoFrameBuffer.push_back(std::move(atlasList));
    ++frameIdx;
  }

  if (m_config.textureOffsetFlag) {
    encodePatchTextureOffset(patchTextureStats);
    applyPatchTextureOffset();
  }
}

auto Encoder::Impl::isRedundantBlock(Common::Vec2i topLeft, Common::Vec2i bottomRight,
                                     uint16_t viewIdx, int32_t frameIdx) const -> bool {
  if (!m_config.patchRedundancyRemoval) {
    return false;
  }
  bottomRight.x() = std::min(topLeft.x() + m_blockSize, bottomRight.x());
  bottomRight.y() = std::min(topLeft.y() + m_blockSize, bottomRight.y());

  for (int32_t y = topLeft.y(); y < bottomRight.y(); ++y) {
    for (int32_t x = topLeft.x(); x < bottomRight.x(); ++x) {
      if (m_nonAggregatedMask[viewIdx](y, x)[frameIdx]) {
        return false;
      }
    }
  }
  return true;
}

namespace {
void adaptPatchStatsToTexture(TextureStats &patchStats, const Common::DeepFrame &view,
                              Common::DeepFrame &atlas, const Common::Vec2i &pView,
                              const Common::Vec2i &pAtlas, Common::Vec3i &colorCorrectionOffset) {
  for (int32_t c = 0; c < 3; ++c) {
    const auto n = 0 < c ? 2 : 1;

    if (pView.x() % n == 0 && pView.y() % n == 0) {
      const auto sample = view.texture.getPlane(c)(pView.y() / n, pView.x() / n);

      patchStats[c] << sample;
      atlas.texture.getPlane(c)(pAtlas.y() / n, pAtlas.x() / n) =
          Common::assertDownCast<uint16_t>(sample + colorCorrectionOffset[c]);
    }
  }
}
} // namespace

// NOLINTNEXTLINE(readability-function-cognitive-complexity,readability-function-size)
auto Encoder::Impl::writePatchInAtlas(const MivBitstream::PatchParams &patchParams,
                                      const Common::DeepFrame &view, Common::DeepFrameList &frame,
                                      int32_t frameIdx, size_t patchIdx) -> TextureStats {
  const auto k = params().vps.indexOf(patchParams.atlasId());
  auto &atlas = frame[k];

  const auto sizeU = patchParams.atlasPatch3dSizeU();
  const auto sizeV = patchParams.atlasPatch3dSizeV();
  const auto posU = patchParams.atlasPatch3dOffsetU();
  const auto posV = patchParams.atlasPatch3dOffsetV();

  const auto &inViewParams = m_transportParams.viewParamsList[patchParams.atlasPatchProjectionId()];
  const auto &outViewParams = params().viewParamsList[patchParams.atlasPatchProjectionId()];

  auto textureStats = TextureStats{};

  PRECONDITION(0 <= posU && posU + sizeU <= inViewParams.ci.ci_projection_plane_width_minus1() + 1);
  PRECONDITION(0 <= posV &&
               posV + sizeV <= inViewParams.ci.ci_projection_plane_height_minus1() + 1);

  for (int32_t vBlock = 0; vBlock < sizeV; vBlock += m_blockSize) {
    for (int32_t uBlock = 0; uBlock < sizeU; uBlock += m_blockSize) {
      const auto viewIdx = params().viewParamsList.indexOf(patchParams.atlasPatchProjectionId());
      const auto redundant = isRedundantBlock({posU + uBlock, posV + vBlock},
                                              {posU + sizeU, posV + sizeV}, viewIdx, frameIdx);
      int32_t yOcc = 0;
      int32_t xOcc = 0;
      for (int32_t v = vBlock; v < vBlock + m_blockSize && v < sizeV; ++v) {
        for (int32_t u = uBlock; u < uBlock + m_blockSize && u < sizeU; ++u) {
          const auto pView = Common::Vec2i{posU + u, posV + v};
          const auto pAtlas = patchParams.viewToAtlas(pView);

          const auto &asme = params().atlas[k].asps.asps_miv_extension();

          if (!asme.asme_embedded_occupancy_enabled_flag() &&
              asme.asme_occupancy_scale_enabled_flag()) {
            yOcc = pAtlas.y() / (asme.asme_occupancy_scale_factor_y_minus1() + 1);
            xOcc = pAtlas.x() / (asme.asme_occupancy_scale_factor_x_minus1() + 1);
          } else {
            yOcc = pAtlas.y();
            xOcc = pAtlas.x();
          }

          if (redundant && m_config.haveGeometry) {
            adaptAtlas(patchParams, atlas, yOcc, xOcc, pView, pAtlas);
            continue;
          }

          if (m_config.haveTexture) {
            adaptPatchStatsToTexture(textureStats, view, atlas, pView, pAtlas,
                                     m_patchColorCorrectionOffset[patchIdx]);
          }

          if (m_config.haveGeometry) {
            auto depth = view.geometry.getPlane(0)(pView.y(), pView.x());

            atlas.occupancy.getPlane(0)(yOcc, xOcc) = true;

            if (depth == 0 && !inViewParams.hasOccupancy && outViewParams.hasOccupancy &&
                asme.asme_max_entity_id() == 0) {
              depth = 1; // Avoid marking valid depth as invalid
            }

            if (depth == 0 && inViewParams.hasOccupancy) {
              atlas.occupancy.getPlane(0)(yOcc, xOcc) = false;
            }

            atlas.geometry.getPlane(0)(pAtlas.y(), pAtlas.x()) = depth;

            if (depth > 0 && params().vps.vps_occupancy_video_present_flag(patchParams.atlasId())) {
              atlas.occupancy.getPlane(0)(yOcc, xOcc) = true;
            }
          }
        }
      }
    }
  }

  return textureStats;
}

void Encoder::Impl::adaptAtlas(const MivBitstream::PatchParams &patchParams,
                               Common::DeepFrame &atlas, int32_t yOcc, int32_t xOcc,
                               const Common::Vec2i &pView, const Common::Vec2i &pAtlas) const {
  atlas.geometry.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;

  if (params().vps.vps_occupancy_video_present_flag(patchParams.atlasId())) {
    atlas.occupancy.getPlane(0)(yOcc, xOcc) = false;
  }
  if (m_config.haveTexture) {
    const auto textureMedVal = Common::medLevel<uint16_t>(atlas.texture.getBitDepth());

    atlas.texture.getPlane(0)(pAtlas.y(), pAtlas.x()) = textureMedVal;
    if ((pView.x() % 2) == 0 && (pView.y() % 2) == 0) {
      atlas.texture.getPlane(1)(pAtlas.y() / 2, pAtlas.x() / 2) = textureMedVal;
      atlas.texture.getPlane(2)(pAtlas.y() / 2, pAtlas.x() / 2) = textureMedVal;
    }
  }
}

} // namespace TMIV::Encoder
