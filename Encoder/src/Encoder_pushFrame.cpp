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

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Encoder {
constexpr auto neutralChroma = TextureFrame::neutralColor();

void Encoder::pushFrame(MVD16Frame sourceViews) {
  if (m_ivs.vme().vme_max_entities_minus1() == 0) {
    pushSingleEntityFrame(move(sourceViews));
  } else {
    pushMultiEntityFrame(move(sourceViews));
  }
}

void Encoder::pushSingleEntityFrame(MVD16Frame sourceViews) {
  auto transportViews = m_viewOptimizer->optimizeFrame(move(sourceViews));
  const auto masks = m_pruner->prune(m_transportIvs, transportViews, m_isBasicView);
  updateNonAggregatedMask(transportViews, masks);
  m_transportViews.push_back(move(transportViews));
  m_aggregator->pushMask(masks);
}

// Atlas dilation
// Visit all pixels
template <typename F> static void forPixels(array<size_t, 2> sizes, F f) {
  for (int i = 0; i < int(sizes[0]); ++i) {
    for (int j = 0; j < int(sizes[1]); ++j) {
      f(i, j);
    }
  }
}

// Visit all pixel neighbors (in between 3 and 8)
template <typename F> static auto forNeighbors(int i, int j, array<size_t, 2> sizes, F f) -> bool {
  const int n1 = max(0, i - 1);
  const int n2 = min(int(sizes[0]), i + 2);
  const int m1 = max(0, j - 1);
  const int m2 = min(int(sizes[1]), j + 2);

  for (int n = n1; n < n2; ++n) {
    for (int m = m1; m < m2; ++m) {
      if (!f(n, m)) {
        return false;
      }
    }
  }
  return true;
}

static auto erode(const Mat<uint8_t> &mask) -> Mat<uint8_t> {
  Mat<uint8_t> result{mask.sizes()};
  forPixels(mask.sizes(), [&](int i, int j) {
    result(i, j) =
        forNeighbors(i, j, mask.sizes(), [&mask](int n, int m) { return mask(n, m) > 0; }) ? 255
                                                                                           : 0;
  });
  return result;
}

static auto dilate(const Mat<uint8_t> &mask) -> Mat<uint8_t> {
  Mat<uint8_t> result{mask.sizes()};
  forPixels(mask.sizes(), [&](int i, int j) {
    result(i, j) =
        forNeighbors(i, j, mask.sizes(), [&mask](int n, int m) { return mask(n, m) == 0; }) ? 0
                                                                                            : 255;
  });
  return result;
}

void Encoder::updateNonAggregatedMask(const MVD16Frame &transportViews, const MaskList &masks) {
  const auto frameId = m_transportViews.size();
  MaskList dilatedMasks = masks; // Atlas dilation

  // Atlas dilation
  for (size_t viewId = 0; viewId < masks.size(); ++viewId) {
    for (int n = 0; n < m_dilationIter; ++n) {
      dilatedMasks[viewId].getPlane(0) = dilate(dilatedMasks[viewId].getPlane(0));
    }
  }

  for (size_t viewId = 0; viewId < masks.size(); ++viewId) {
    const auto height = transportViews[viewId].texture.getHeight();
    const auto width = transportViews[viewId].texture.getWidth();

    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        if (dilatedMasks[viewId].getPlane(0)(i, j) != 0) {
          m_nonAggregatedMask[viewId](i, j)[frameId] = true;
        }
      }
    }
  }
}

void Encoder::pushMultiEntityFrame(MVD16Frame sourceViews) {
  auto transportViews = m_viewOptimizer->optimizeFrame(move(sourceViews));

  MaskList mergedMasks;
  for (const auto &transportView : transportViews) {
    mergedMasks.emplace_back(transportView.texture.getWidth(), transportView.texture.getHeight());
  }

  for (auto entityId = m_entityEncRange[0]; entityId < m_entityEncRange[1]; entityId++) {
    cout << "Processing entity " << entityId << '\n';

    const auto transportEntityViews = entitySeparator(transportViews, entityId);
    auto masks = m_pruner->prune(m_transportIvs, transportEntityViews, m_isBasicView);
    updateMasks(transportEntityViews, masks);
    aggregateEntityMasks(masks, entityId);
    mergeMasks(mergedMasks, masks);
  }

  updateNonAggregatedMask(transportViews, mergedMasks);
  m_transportViews.push_back(move(transportViews));
  m_aggregator->pushMask(mergedMasks);
}

auto Encoder::yuvSampler(const EntityMapList &in) -> vector<Frame<YUV420P16>> {
  vector<Frame<YUV420P16>> outYuvAll;
  for (const auto &viewId : in) {
    Frame<YUV420P16> outYuv(int(viewId.getWidth()), int(viewId.getHeight()));
    const auto width = viewId.getWidth();
    const auto height = viewId.getHeight();
    int step = 1;
    for (int k = 0; k < 3; ++k) {
      if (k != 0) {
        step = 2;
      }
      int rowIndex = 0;
      for (int i = 0; i != height; i = i + step) {
        int colIndex = 0;
        for (int j = 0; j != width; j = j + step) {
          outYuv.getPlane(k)(rowIndex, colIndex) = viewId.getPlane(0)(i, j);
          colIndex++;
        }
        rowIndex++;
      }
    }
    outYuvAll.push_back(outYuv);
  }
  return outYuvAll;
}

void Encoder::mergeMasks(MaskList &mergedMasks, MaskList masks) {
  for (size_t viewId = 0; viewId < mergedMasks.size(); viewId++) {
    for (size_t i = 0; i < mergedMasks[viewId].getPlane(0).size(); i++) {
      if (masks[viewId].getPlane(0)[i] != uint8_t(0)) {
        mergedMasks[viewId].getPlane(0)[i] = masks[viewId].getPlane(0)[i];
      }
    }
  }
}

void Encoder::updateMasks(const MVD16Frame &views, MaskList &masks) {
  for (size_t viewId = 0; viewId < views.size(); viewId++) {
    for (size_t i = 0; i < masks[viewId].getPlane(0).size(); i++) {
      if ((views[viewId].texture.getPlane(0)[i] == neutralChroma) &&
          (views[viewId].depth.getPlane(0)[i] == uint16_t(0))) {
        masks[viewId].getPlane(0)[i] = uint8_t(0);
      }
    }
  }
}

void Encoder::aggregateEntityMasks(MaskList &masks, uint16_t entityId) {
  if (int(m_aggregatedEntityMask.size()) < m_entityEncRange[1] - m_entityEncRange[0]) {
    m_aggregatedEntityMask.push_back(masks);
  } else {
    for (size_t i = 0; i < masks.size(); i++) {
      transform(m_aggregatedEntityMask[entityId - m_entityEncRange[0]][i].getPlane(0).begin(),
                m_aggregatedEntityMask[entityId - m_entityEncRange[0]][i].getPlane(0).end(),
                masks[i].getPlane(0).begin(),
                m_aggregatedEntityMask[entityId - m_entityEncRange[0]][i].getPlane(0).begin(),
                [](auto v1, auto v2) { return max(v1, v2); });
    }
  }
}

auto Encoder::entitySeparator(const MVD16Frame &transportViews, uint16_t entityId) -> MVD16Frame {
  // Initalize entityViews
  MVD16Frame entityViews;
  for (const auto &transportView : transportViews) {
    TextureDepth16Frame entityView = {
        TextureFrame(transportView.texture.getWidth(), transportView.texture.getHeight()),
        Depth16Frame(transportView.depth.getWidth(), transportView.depth.getHeight())};
    entityViews.push_back(move(entityView));
  }
  EntityMapList entityMaps;
  for (const auto &transportView : transportViews) {
    entityMaps.push_back(transportView.entities);
  }

  auto entityMapsYUV = yuvSampler(entityMaps);

  for (size_t viewId = 0; viewId < transportViews.size(); viewId++) {
    for (int planeId = 0; planeId < transportViews[viewId].texture.getNumberOfPlanes();
         planeId++) {                                                     //
      transform(transportViews[viewId].texture.getPlane(planeId).begin(), // i's
                transportViews[viewId].texture.getPlane(planeId).end(),   //
                entityMapsYUV[viewId].getPlane(planeId).begin(),          // j's
                entityViews[viewId].texture.getPlane(planeId).begin(),    // result
                [=](auto i, auto j) { return (j == entityId) ? i : neutralChroma; });
    }
    transform(transportViews[viewId].depth.getPlane(0).begin(), // i's
              transportViews[viewId].depth.getPlane(0).end(),   //
              entityMaps[viewId].getPlane(0).begin(),           // j's
              entityViews[viewId].depth.getPlane(0).begin(),    // result
              [=](auto i, auto j) { return (j == entityId) ? i : uint16_t(0); });
  }

  return entityViews;
}
} // namespace TMIV::Encoder
