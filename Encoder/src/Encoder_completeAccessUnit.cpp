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

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Encoder {
auto Encoder::scaleGeometryDynamicRange() -> void {
  bool lowDepthQuality = m_params.vps.vps_miv_extension().vme_depth_low_quality_flag();
  int numOfFrames = m_transportViews.size();
  int numOfViews = m_transportViews[0].size();

  for (int v = 0; v < numOfViews; v++) {
    int minDepthMapValWithinGOP = 65535;
    int maxDepthMapValWithinGOP = 0;

    for (int f = 0; f < numOfFrames; f++) {
      for (const auto geometry : m_transportViews[f][v].depth.getPlane(0)) {
        if (geometry < minDepthMapValWithinGOP) {
          minDepthMapValWithinGOP = geometry;
        }
        if (geometry > maxDepthMapValWithinGOP) {
          maxDepthMapValWithinGOP = geometry;
        }
      }
    }

    for (int f = 0; f < numOfFrames; f++) {
      for (auto &geometry : m_transportViews[f][v].depth.getPlane(0)) {
        geometry = (geometry + 0.5 - minDepthMapValWithinGOP) /
                   (maxDepthMapValWithinGOP - minDepthMapValWithinGOP) * 65535.0;
        if (lowDepthQuality) {
          geometry /= 2;
        }
      }
    }

    double NDH_orig = m_params.norm_disp_high_orig[v];
    double NDL_orig = m_params.norm_disp_low_orig[v];

    double NDH = maxDepthMapValWithinGOP / 65535.0 * (NDH_orig - NDL_orig) + NDL_orig;
    double NDL = minDepthMapValWithinGOP / 65535.0 * (NDH_orig - NDL_orig) + NDL_orig;

    if (lowDepthQuality) {
      NDH = 2 * NDH - NDL;
    }

    m_params.viewParamsList[v].dq.dq_norm_disp_high(NDH);
    m_params.viewParamsList[v].dq.dq_norm_disp_low(NDL);
  }
} // namespace TMIV::Encoder

auto Encoder::completeAccessUnit() -> const EncoderParams & {
  m_aggregator->completeAccessUnit();
  const auto &aggregatedMask = m_aggregator->getAggregatedMask();

  updateAggregationStatistics(aggregatedMask);

  scaleGeometryDynamicRange();

  if (m_params.vme().vme_max_entities_minus1() > 0) {
    m_packer->updateAggregatedEntityMasks(m_aggregatedEntityMask);
  }

  m_params.patchParamsList =
      m_packer->pack(m_params.atlasSizes(), aggregatedMask, m_transportParams.viewParamsList);

  constructVideoFrames();

  const auto &paramsQuantized = m_geometryQuantizer->transformParams(m_params);
  const auto &paramsScaled = m_geometryDownscaler.transformParams(paramsQuantized);
  return paramsScaled;
}

void Encoder::updateAggregationStatistics(const MaskList &aggregatedMask) {
  const auto lumaSamplesPerFrame = accumulate(
      aggregatedMask.begin(), aggregatedMask.end(), size_t{}, [](size_t sum, const auto &mask) {
        return sum + 2 * count_if(mask.getPlane(0).begin(), mask.getPlane(0).end(),
                                  [](auto x) { return x > 0; });
      });
  cout << "Aggregated luma samples per frame is " << (1e-6 * lumaSamplesPerFrame) << "M\n";
  m_maxLumaSamplesPerFrame = max(m_maxLumaSamplesPerFrame, lumaSamplesPerFrame);
}

void Encoder::constructVideoFrames() {
  int frame = 0;
  for (const auto &views : m_transportViews) {
    MVD16Frame atlasList;

    for (uint8_t i = 0; i <= m_params.vps.vps_atlas_count_minus1(); ++i) {
      const auto frameWidth = m_params.vps.vps_frame_width(i);
      const auto frameHeight = m_params.vps.vps_frame_height(i);
      TextureDepth16Frame frame = {TextureFrame(frameWidth, frameHeight),
                                   Depth16Frame(frameWidth, frameHeight)};
      frame.texture.fillNeutral();
      frame.depth.fillZero();
      atlasList.push_back(move(frame));
    }

    for (const auto &patch : m_params.patchParamsList) {
      const auto &view = views[patch.pduViewId()];
      if (m_params.vme().vme_max_entities_minus1() > 0) {
        MVD16Frame tempViews;
        tempViews.push_back(view);
        const auto &entityViews = entitySeparator(tempViews, *patch.pduEntityId());
        writePatchInAtlas(patch, entityViews[0], atlasList, frame);
      } else {
        writePatchInAtlas(patch, view, atlasList, frame);
      }
    }
    m_videoFrameBuffer.push_back(move(atlasList));
    frame++;
  }
}

void Encoder::writePatchInAtlas(const PatchParams &patchParams, const TextureDepth16Frame &view,
                                MVD16Frame &atlas, int frameId) {
  auto &currentAtlas = atlas[patchParams.vuhAtlasId];

  auto &textureAtlasMap = currentAtlas.texture;
  auto &depthAtlasMap = currentAtlas.depth;

  const auto &textureViewMap = view.texture;
  const auto &depthViewMap = view.depth;
  int w = patchParams.pduViewSize().x();
  int h = patchParams.pduViewSize().y();
  int xM = patchParams.pduViewPos().x();
  int yM = patchParams.pduViewPos().y();

  const auto &inViewParams = m_transportParams.viewParamsList[patchParams.pduViewId()];
  const auto &outViewParams = m_params.viewParamsList[patchParams.pduViewId()];

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
          if (m_nonAggregatedMask[patchParams.pduViewId()](dy + yM, dx + xM)[frameId]) {
            isAggregatedMaskBlockNonEmpty = true;
            break;
          }
        }
        if (isAggregatedMaskBlockNonEmpty) {
          break;
        }
      }

      for (int dy = dyAligned; dy < dyAligned + m_blockSize; dy++) {
        for (int dx = dxAligned; dx < dxAligned + m_blockSize; dx++) {
          Vec2i pView = {xM + dx, yM + dy};
          Vec2i pAtlas = patchParams.viewToAtlas(pView);

          if (pView.y() >= textureViewMap.getHeight() || pView.x() >= textureViewMap.getWidth() ||
              pAtlas.y() >= textureAtlasMap.getHeight() ||
              pAtlas.x() >= textureAtlasMap.getWidth() || pView.y() < 0 || pView.x() < 0 ||
              pAtlas.y() < 0 || pAtlas.x() < 0) {
            continue;
          }

          if (!isAggregatedMaskBlockNonEmpty) {
            depthAtlasMap.getPlane(0)(pAtlas.y(), pAtlas.x()) = 0;
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
        }
      }
    }
  }
}
} // namespace TMIV::Encoder
