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

#include <TMIV/MivBitstream/DepthOccupancyTransform.h>

#include <fmt/format.h>

#include <cassert>

namespace TMIV::MivBitstream {
DepthTransform::DepthTransform(const DepthQuantization &dq, uint32_t bitDepth)
    : m_normDispLow{dq.dq_norm_disp_low()}
    , m_normDispHigh{dq.dq_norm_disp_high()}
    , m_bitDepth{bitDepth}
    , m_maxSampleValue{Common::maxLevel(bitDepth)} {
  PRECONDITION(0 < m_bitDepth);

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

  m_viewPivotCount = dq.dq_pivot_count_minus1() + 1;
  m_quantizationLaw = dq.dq_quantization_law();

  if (m_quantizationLaw == 2) {
    m_normDispMap.resize(m_viewPivotCount + 2);
    m_normDispMap[0] = m_normDispLow;
    m_normDispMap[m_viewPivotCount + 1] = m_normDispHigh;

    for (int32_t i = 0; i < m_viewPivotCount; i++) {
      m_normDispMap[i + 1] = dq.dq_pivot_norm_disp(i);
    }

    m_normDispMax = m_normDispHigh;
    m_normDispMax =
        (m_normDispMax + m_normDispMap[0]) / 2; // NOTE[FT]: why is this halving needed ?

    m_normDispInterval =
        (m_normDispMax - m_normDispMap[0]) / static_cast<float>(m_viewPivotCount + 1);
  }
}

DepthTransform::DepthTransform(const DepthQuantization &dq, const PatchParams &patchParams,
                               uint32_t bitDepth)
    : DepthTransform{dq, bitDepth} {
  m_atlasPatch3dOffsetD = patchParams.atlasPatch3dOffsetD();
  m_atlasPatch3dRangeD = patchParams.atlasPatch3dRangeD();
}

auto DepthTransform::expandNormDisp(Common::SampleValue x) const -> float {
  const auto level = static_cast<float>(m_atlasPatch3dOffsetD + std::min(x, m_atlasPatch3dRangeD)) /
                     static_cast<float>(m_maxSampleValue);

  if (m_quantizationLaw == 2) {
    float normDisp = m_normDispLow + (m_normDispHigh - m_normDispLow) * level;

    for (int32_t i = 0; i <= m_viewPivotCount; i++) {
      if (normDisp <= m_normDispMap[i + 1]) {
        const auto x1 = m_normDispMap[i];
        const auto x2 = m_normDispMap[i + 1];
        const auto mappedIntervalSize = x2 - x1;
        float orgStartMap = m_normDispMap[0] + static_cast<float>(i) * m_normDispInterval;
        normDisp = std::max(m_minNormDisp, orgStartMap + (normDisp - x1) * m_normDispInterval /
                                                             mappedIntervalSize);
        break;
      }
    }
    return normDisp;
  }

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
    float level{};

    if (m_quantizationLaw == 0) {
      level = (x - m_normDispLow) / (m_normDispHigh - m_normDispLow);
    } else if (m_quantizationLaw == 2) {
      uint8_t i = std::clamp(static_cast<uint8_t>((x - m_normDispMap[0]) / m_normDispInterval),
                             uint8_t{}, m_viewPivotCount);
      auto orgStartMap =
          static_cast<double>(m_normDispMap[0] + static_cast<double>(i) * m_normDispInterval);
      const auto x1 = static_cast<double>(m_normDispMap[i]);
      const auto x2 = static_cast<double>(m_normDispMap[i + 1]);
      const auto mappedIntervalSize = x2 - x1;
      double normDisp =
          std::clamp(x1 + (x - orgStartMap) * (mappedIntervalSize) / m_normDispInterval,
                     static_cast<double>(m_normDispMap[0]),
                     static_cast<double>(m_normDispMap[m_viewPivotCount + 1]));
      level = static_cast<float>((normDisp - m_normDispLow) / (m_normDispHigh - m_normDispLow));
    }
    const auto quantized =
        std::max(Common::quantizeValue(level, m_bitDepth), m_atlasPatch3dOffsetD);
    return std::clamp(quantized - m_atlasPatch3dOffsetD, minLevel, m_atlasPatch3dRangeD);
  }
  return 0;
}

auto DepthTransform::minNormDisp() const -> float { return m_minNormDisp; }
} // namespace TMIV::MivBitstream