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

#include "EncoderImpl.h"

#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/MivBitstream/Formatters.h>

namespace TMIV::Encoder {
void Encoder::Impl::pushFrame(Common::DeepFrameList sourceViews) {
  if (m_config.maxEntityId == 0) {
    pushSingleEntityFrame(std::move(sourceViews));
  } else {
    pushMultiEntityFrame(std::move(sourceViews));
  }
}

void Encoder::Impl::pushSingleEntityFrame(Common::DeepFrameList sourceViews) {
  auto transportViews = m_viewOptimizer->optimizeFrame(std::move(sourceViews));
  const auto masks = m_pruner->prune(m_transportParams.viewParamsList, transportViews);
  m_transportViews.push_back(std::move(transportViews));
  m_aggregator->pushMask(masks);
}

namespace {
// Atlas dilation
// Visit all pixels
template <typename F> void forPixels(std::array<size_t, 2> sizes, F f) {
  for (int32_t i = 0; i < static_cast<int32_t>(sizes[0]); ++i) {
    for (int32_t j = 0; j < static_cast<int32_t>(sizes[1]); ++j) {
      f(i, j);
    }
  }
}

// Visit all pixel neighbors (in between 3 and 8)
template <typename F>
auto forNeighbors(int32_t i, int32_t j, std::array<size_t, 2> sizes, F f) -> bool {
  const int32_t n1 = std::max(0, i - 1);
  const int32_t n2 = std::min(static_cast<int32_t>(sizes[0]), i + 2);
  const int32_t m1 = std::max(0, j - 1);
  const int32_t m2 = std::min(static_cast<int32_t>(sizes[1]), j + 2);

  for (int32_t n = n1; n < n2; ++n) {
    for (int32_t m = m1; m < m2; ++m) {
      if (!f(n, m)) {
        return false;
      }
    }
  }
  return true;
}
} // namespace

void Encoder::Impl::pushMultiEntityFrame(Common::DeepFrameList sourceViews) {
  auto transportViews = m_viewOptimizer->optimizeFrame(std::move(sourceViews));

  Common::FrameList<uint8_t> mergedMasks;
  for (const auto &transportView : transportViews) {
    mergedMasks.emplace_back().createY(transportView.texture.getSize());
  }

  for (auto entityId = m_config.entityEncRange[0]; entityId < m_config.entityEncRange[1];
       entityId++) {
    Common::logInfo("Processing entity {}", entityId);

    const auto transportEntityViews = entitySeparator(transportViews, entityId);
    auto masks = m_pruner->prune(m_transportParams.viewParamsList, transportEntityViews);
    updateMasks(transportEntityViews, masks);
    aggregateEntityMasks(masks, entityId);
    mergeMasks(mergedMasks, masks);
  }

  m_transportViews.push_back(std::move(transportViews));
  m_aggregator->pushMask(mergedMasks);
}

auto Encoder::Impl::yuvSampler(const Common::FrameList<> &in) -> Common::FrameList<> {
  Common::FrameList<> outYuvAll;

  for (const auto &entityMap : in) {
    auto outYuv = Common::Frame<>::yuv420(entityMap.getSize(), entityMap.getBitDepth());
    const auto width = entityMap.getWidth();
    const auto height = entityMap.getHeight();
    int32_t step = 1;
    for (int32_t k = 0; k < 3; ++k) {
      if (k != 0) {
        step = 2;
      }
      int32_t rowIdx = 0;
      for (int32_t i = 0; i != height; i = i + step) {
        int32_t colIdx = 0;
        for (int32_t j = 0; j != width; j = j + step) {
          outYuv.getPlane(k)(rowIdx, colIdx) = entityMap.getPlane(0)(i, j);
          colIdx++;
        }
        rowIdx++;
      }
    }
    outYuvAll.push_back(outYuv);
  }
  return outYuvAll;
}

void Encoder::Impl::mergeMasks(Common::FrameList<uint8_t> &mergedMasks,
                               Common::FrameList<uint8_t> masks) {
  for (size_t viewIdx = 0; viewIdx < mergedMasks.size(); viewIdx++) {
    for (size_t i = 0; i < mergedMasks[viewIdx].getPlane(0).size(); i++) {
      if (masks[viewIdx].getPlane(0)[i] != uint8_t{}) {
        mergedMasks[viewIdx].getPlane(0)[i] = masks[viewIdx].getPlane(0)[i];
      }
    }
  }
}

void Encoder::Impl::updateMasks(const Common::DeepFrameList &views,
                                Common::FrameList<uint8_t> &masks) {
  for (size_t viewIdx = 0; viewIdx < views.size(); viewIdx++) {
    for (size_t i = 0; i < masks[viewIdx].getPlane(0).size(); i++) {
      if (views[viewIdx].geometry.getPlane(0)[i] == uint16_t{}) {
        masks[viewIdx].getPlane(0)[i] = uint8_t{};
      }
    }
  }
}

void Encoder::Impl::aggregateEntityMasks(Common::FrameList<uint8_t> &masks,
                                         Common::SampleValue entityId) {
  if (m_aggregatedEntityMask.size() < m_config.entityEncRange[1] - m_config.entityEncRange[0]) {
    m_aggregatedEntityMask.push_back(masks);
  } else {
    for (size_t i = 0; i < masks.size(); i++) {
      auto &entityMaskPlane =
          m_aggregatedEntityMask[entityId - m_config.entityEncRange[0]][i].getPlane(0);
      std::transform(entityMaskPlane.begin(), entityMaskPlane.end(), masks[i].getPlane(0).begin(),
                     entityMaskPlane.begin(), [](auto v1, auto v2) { return std::max(v1, v2); });
    }
  }
}

auto Encoder::Impl::entitySeparator(const Common::DeepFrameList &transportViews,
                                    Common::SampleValue entityId) -> Common::DeepFrameList {
  // Initalize entityViews
  Common::DeepFrameList entityViews;
  for (const auto &transportView : transportViews) {
    Common::DeepFrame entityView = {
        Common::Frame<>::yuv420(transportView.texture.getSize(),
                                transportView.texture.getBitDepth()),
        Common::Frame<>::lumaOnly(transportView.geometry.getSize(),
                                  transportView.geometry.getBitDepth())};
    entityViews.push_back(std::move(entityView));
  }
  Common::FrameList<> entityMaps;
  for (const auto &transportView : transportViews) {
    entityMaps.push_back(transportView.entities);
  }

  auto entityMapsYUV = yuvSampler(entityMaps);

  for (size_t viewIdx = 0; viewIdx < transportViews.size(); viewIdx++) {
    const auto neutralColor = entityViews[viewIdx].texture.neutralValue();

    for (int32_t planeIdx = 0; planeIdx < 3; ++planeIdx) {
      std::transform(transportViews[viewIdx].texture.getPlane(planeIdx).begin(),
                     transportViews[viewIdx].texture.getPlane(planeIdx).end(),
                     entityMapsYUV[viewIdx].getPlane(planeIdx).begin(),
                     entityViews[viewIdx].texture.getPlane(planeIdx).begin(),
                     [=](auto i, auto j) { return (j == entityId) ? i : neutralColor; });
    }
    std::transform(transportViews[viewIdx].geometry.getPlane(0).begin(),
                   transportViews[viewIdx].geometry.getPlane(0).end(),
                   entityMaps[viewIdx].getPlane(0).begin(),
                   entityViews[viewIdx].geometry.getPlane(0).begin(),
                   [=](auto i, auto j) { return (j == entityId) ? i : uint16_t{}; });
  }

  return entityViews;
}
} // namespace TMIV::Encoder
