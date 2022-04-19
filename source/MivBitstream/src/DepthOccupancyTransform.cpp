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

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <fmt/format.h>

#include <cassert>

namespace TMIV::MivBitstream {
DepthTransform::DepthTransform(const DepthQuantization &dq, uint32_t bitDepth)
    : m_normDispLow{dq.dq_norm_disp_low()}
    , m_normDispHigh{dq.dq_norm_disp_high()}
    , m_bitDepth{bitDepth} {
  if (!std::isfinite(m_normDispLow) || !std::isfinite(m_normDispHigh) ||
      m_normDispLow == m_normDispHigh || std::max(m_normDispLow, m_normDispHigh) <= 0.F) {
    throw std::runtime_error(
        fmt::format("Invalid normalized disparity range [{}, {}]", m_normDispLow, m_normDispHigh));
  }

  const auto [far, near] = std::minmax(m_normDispLow, m_normDispHigh);
  const auto maxLevel = std::ldexp(1.F, static_cast<int32_t>(bitDepth)) - 1.F;
  const auto lowestLevel = std::ceil(std::nextafter(-far / (near - far) * maxLevel, INFINITY));
  m_minNormDisp = far + (near - far) * (lowestLevel / maxLevel);
  ASSERT(0 < m_minNormDisp);

#if ENABLE_M57419
  m_intervalNum = dq.dq_interval_num();
  m_quantizationLaw = dq.dq_quantization_law();

  if (m_quantizationLaw != static_cast<uint8_t>(0)) {
    m_normDispMap.resize(m_intervalNum + 1);
    for (int i = 0; i < m_intervalNum + 1; i++) {
      m_normDispMap[i] = dq.dq_norm_disp_map(i);
    }

    m_normDispMax = m_normDispHigh;
    if (m_quantizationLaw == static_cast<uint8_t>(2)) {
      m_normDispMax = (m_normDispMax + m_normDispMap[0]) / 2;
    }

    m_normDispInterval = (m_normDispMax - m_normDispMap[0]) / static_cast<float>(m_intervalNum);
  }
#endif
}

DepthTransform::DepthTransform(const DepthQuantization &dq, const PatchParams &patchParams,
                               uint32_t bitDepth)
    : DepthTransform{dq, bitDepth} {
  m_depthStart = patchParams.atlasPatch3dOffsetD();
  m_depthEnd = m_depthStart + patchParams.atlasPatch3dRangeD();
}

auto DepthTransform::expandNormDisp(Common::SampleValue x) const -> float {
  const auto level = Common::expandValue(std::clamp(x, m_depthStart, m_depthEnd), m_bitDepth);

#if ENABLE_M57419
  if (m_quantizationLaw == 1) {
    float normDisp = m_normDispLow + (m_normDispHigh - m_normDispLow) * level;

    for (int i = 1; i <= m_intervalNum; i++) {
      if (normDisp <= m_normDispMap[i]) {
        float in = m_normDispMap[i - 1];
        float map = m_normDispMap[0] + static_cast<float>(i - 1) * m_normDispInterval;
        float inD = m_normDispMap[i] - in;
        float mapD = m_normDispInterval;
        normDisp = std::max(m_minNormDisp, (normDisp - in) * mapD / inD + map);
        break;
      }
    }
    return normDisp;
  }
#endif

  return std::max(m_minNormDisp, m_normDispLow + (m_normDispHigh - m_normDispLow) * level);
}

auto DepthTransform::expandDepth(Common::SampleValue x) const -> float {
  return 1.F / expandNormDisp(x);
}

auto DepthTransform::expandDepth(const Common::Mat<> &matrix) const -> Common::Mat<float> {
  auto depth = Common::Mat<float>(matrix.sizes());
  std::transform(std::begin(matrix), std::end(matrix), std::begin(depth),
                 [this](auto x) { return expandDepth(x); });
  return depth;
}

auto DepthTransform::expandDepth(const Common::Frame<> &frame) const -> Common::Mat<float> {
  PRECONDITION(frame.getBitDepth() == m_bitDepth);

  return expandDepth(frame.getPlane(0));
}

auto DepthTransform::quantizeNormDisp(float x, Common::SampleValue minLevel) const
    -> Common::SampleValue {
  if (x > 0.F) {
#if ENABLE_M54719
    float level{};

    if (m_quantizationLaw == 0) {
      level = (x - m_normDispLow) / (m_normDispHigh - m_normDispLow);
    } else if (m_quantizationLaw == 1) {
      int interval_index =
          std::clamp((int)((x - m_normDispMap[0]) / m_normDispInterval), 0, m_intervalNum - 1);

      auto in = (double)(m_normDispMap[0] + (double)interval_index * m_normDispInterval);
      auto map = (double)m_normDispMap[interval_index];
      double inD = m_normDispInterval;
      double mapD = m_normDispMap[interval_index + 1] - map;

      double normDisp = std::clamp((x - in) * mapD / inD + map, (double)m_normDispMap[0],
                                   (double)m_normDispMap[m_intervalNum]);
      level = (float)((normDisp - m_normDispLow) / (m_normDispHigh - m_normDispLow));
    }
#else
    const auto level = (x - m_normDispLow) / (m_normDispHigh - m_normDispLow);
#endif
    return std::max(minLevel, Common::quantizeValue(level, m_bitDepth));
  }
  return 0;
}

auto DepthTransform::minNormDisp() const -> float { return m_minNormDisp; }
} // namespace TMIV::MivBitstream