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

#include "PiecewiseLinearDepthScaling.h"
#include "EncoderImpl.h"

#include <TMIV/Common/Common.h>

#include <algorithm>
#include <numeric>

namespace TMIV::Encoder {
#if ENABLE_M57419
auto m57419_edgeDetection(std::vector<std::vector<int32_t>> geometryUnit, double line_thres)
    -> bool {
  bool isEdgeSample = false;

  int32_t gv = abs(2 * geometryUnit[1][1] - geometryUnit[0][1] - geometryUnit[2][1]);
  int32_t gh = abs(2 * geometryUnit[1][1] - geometryUnit[1][0] - geometryUnit[1][2]);
  int32_t gd0 = abs(2 * geometryUnit[1][1] - geometryUnit[0][0] - geometryUnit[2][2]);
  int32_t gd1 = abs(2 * geometryUnit[1][1] - geometryUnit[0][2] - geometryUnit[2][0]);

  double gvh_max = std::max(gv, gh);
  double gvh_min = std::min(gv, gh);
  double gd_max = std::max(gd0, gd1);
  double gd_min = std::min(gd0, gd1);

  if ((gvh_max > line_thres * gvh_min) || (gd_max > line_thres * gd_min)) {
    if ((gvh_min != 0 && gd_min != 0) || (gvh_min == 0 && gvh_max > line_thres) ||
        (gd_min == 0 && gd_max > line_thres)) {
      isEdgeSample = true;
    }
  }
  return isEdgeSample;
}

auto m57419_normalizeHistogram(const std::vector<int32_t> &histEdge, int32_t piece_num,
                               bool lowDepthQuality, int32_t minDepthVal, int32_t maxDepthVal)
    -> std::vector<double> {
  double total_histEdge = std::accumulate(histEdge.begin(), histEdge.end(), 0);

  std::vector<double> norm_histEdge;
  norm_histEdge.assign(piece_num, 0);
  for (int32_t i = 0; i < piece_num; i++) {
    norm_histEdge[i] = static_cast<double>(histEdge[i]) / total_histEdge;
  }

  double maxProb_ori = *max_element(norm_histEdge.begin(), norm_histEdge.end());
  double minProb_ori = *min_element(norm_histEdge.begin(), norm_histEdge.end());

  double restrict_factor = 0.25;
  double maxProb_restrict = 0.;
  double minProb_restrict = 0.;
  if (lowDepthQuality) {
    maxProb_restrict = static_cast<double>(1) / static_cast<double>(piece_num);
    minProb_restrict =
        static_cast<double>(1 - 2 * restrict_factor) / static_cast<double>(piece_num);
  } else {
    maxProb_restrict = static_cast<double>(1 + restrict_factor) / static_cast<double>(piece_num);
    minProb_restrict = static_cast<double>(1 - restrict_factor) / static_cast<double>(piece_num);
  }

  double alpha = (minProb_restrict * maxProb_ori) - (maxProb_restrict * minProb_ori);
  for (int32_t i = 0; i < piece_num; i++) {
    norm_histEdge[i] = ((maxProb_restrict - minProb_restrict) * norm_histEdge[i] + alpha) /
                       (maxProb_ori - minProb_ori);
  }

  total_histEdge = 0.;
  for (int32_t i = 0; i < piece_num; i++) {
    total_histEdge += norm_histEdge[i];
  }

  if (lowDepthQuality && (total_histEdge <= 1.0)) {
    total_histEdge = 1.0;
  }

  double accum_histEdge = 0.;
  for (int32_t i = 0; i < piece_num; i++) {
    accum_histEdge += norm_histEdge[i];
    norm_histEdge[i] = accum_histEdge / total_histEdge;
  }

  std::vector<double> mapped_pivot;
  mapped_pivot.assign(piece_num + 1, 0.);
  mapped_pivot[0] = minDepthVal;
  for (int32_t i = 1; i <= piece_num; i++) {
    mapped_pivot[i] =
        minDepthVal + static_cast<int32_t>((maxDepthVal - minDepthVal) * norm_histEdge[i - 1]);
  }
  return mapped_pivot;
}

auto m57419_depthMapping(int32_t minDepthVal, int32_t maxDepthVal, int32_t piece_num,
                         uint16_t inGeometry, const std::vector<double> &mapped_pivot,
                         bool lowDepthQuality) -> uint16_t {
  static constexpr int32_t maxValue = Common::maxLevel(Common::sampleBitDepth);
  static constexpr auto maxValD = static_cast<double>(maxValue);

  double depthStep = static_cast<double>(maxDepthVal - minDepthVal) / piece_num;
  int32_t depthStep_idx =
      std::clamp(static_cast<int32_t>((static_cast<double>(inGeometry - minDepthVal)) / depthStep),
                 0, piece_num - 1);
  auto in = static_cast<uint16_t>((depthStep_idx * depthStep) + minDepthVal);
  double map = mapped_pivot[depthStep_idx];
  double mapD = mapped_pivot[depthStep_idx + 1] - mapped_pivot[depthStep_idx];
  double temp_geo = static_cast<double>(inGeometry - in) * mapD / depthStep + map;
  uint16_t outGeometry =
      std::clamp(static_cast<uint16_t>((temp_geo + 0.5 - minDepthVal) /
                                       (static_cast<double>(maxDepthVal) - minDepthVal) * maxValD),
                 static_cast<uint16_t>(0), static_cast<uint16_t>(maxValD));

  if (lowDepthQuality) {
    outGeometry = outGeometry / 2;
  }
  return outGeometry;
}

auto Encoder::Impl::m57419_makeHistogram(int32_t piece_num, size_t numOfFrames, size_t v,
                                         int32_t minDepthVal, int32_t maxDepthVal)
    -> std::vector<int32_t> {
  double line_thres = m_config.m57419_edgeThreshold;
  double interval = static_cast<double>(maxDepthVal - minDepthVal) / piece_num;

  std::vector<int32_t> histEdge;
  histEdge.assign(piece_num, 0);

  for (size_t f = 0; f < numOfFrames; f++) {
    int32_t heightOfView = m_transportViews[f][v].geometry.getHeight();
    int32_t widthOfView = m_transportViews[f][v].geometry.getWidth();

    for (int32_t i = 1; i < heightOfView - 1; ++i) {
      for (int32_t j = 1; j < widthOfView - 1; ++j) {
        std::vector<std::vector<int32_t>> geometryUnit(3, std::vector(3, 0));
        for (int32_t ki = 0; ki <= 2; ++ki) {
          for (int32_t kj = 0; kj <= 2; ++kj) {
            int32_t ui = i - 1 + ki;
            int32_t uj = i - 1 + kj;
            geometryUnit[ki][kj] = m_transportViews[f][v].geometry.getPlane(0)(ui, uj);
          }
        }
        if (m_nonAggregatedMask[v](i, j)[f] && m57419_edgeDetection(geometryUnit, line_thres)) {
          int32_t interval_idx =
              std::clamp(static_cast<int32_t>(
                             (static_cast<double>(geometryUnit[1][1] - minDepthVal)) / interval),
                         0, piece_num - 1);
          histEdge[interval_idx]++;
        }
      }
    }
  }
  return histEdge;
}

auto Encoder::Impl::m57419_piecewiseLinearScaleGeometryDynamicRange(size_t numOfFrames, size_t v,
                                                                    int32_t minDepthMapValWithinGOP,
                                                                    int32_t maxDepthMapValWithinGOP,
                                                                    bool lowDepthQuality)
    -> std::vector<double> {
  int32_t piece_num = m_config.m57419_intervalNumber;
  VERIFY(0 < piece_num);

  std::vector<int32_t> histEdge;
  std::vector<double> mapped_pivot;
  histEdge = m57419_makeHistogram(piece_num, numOfFrames, v, minDepthMapValWithinGOP,
                                  maxDepthMapValWithinGOP);
  mapped_pivot = m57419_normalizeHistogram(histEdge, piece_num, lowDepthQuality,
                                           minDepthMapValWithinGOP, maxDepthMapValWithinGOP);
  for (size_t f = 0; f < numOfFrames; f++) {
    for (auto &geometry : m_transportViews[f][v].geometry.getPlane(0)) {
      uint16_t inGeometry = geometry;
      geometry = m57419_depthMapping(minDepthMapValWithinGOP, maxDepthMapValWithinGOP, piece_num,
                                     inGeometry, mapped_pivot, lowDepthQuality);
    }
  }
  return mapped_pivot;
}
#endif
} // namespace TMIV::Encoder
