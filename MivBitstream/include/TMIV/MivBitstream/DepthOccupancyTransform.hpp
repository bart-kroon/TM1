/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ISO/IEC
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

#ifndef _TMIV_MIVBITSTREAM_DEPTHOCCUPANCYTRANSFORM_H_
#error "Include the .h, not the .hpp"
#endif

#include <TMIV/Common/Common.h>

namespace TMIV::MivBitstream {
namespace impl {
// An implementation-specific minimum normalized disparity
constexpr auto minNormDisp = 1e-3F; // 1 kilometer
} // namespace impl

inline OccupancyTransform::OccupancyTransform(const ViewParams &viewParams) {
  m_threshold = viewParams.dq.dq_depth_occ_map_threshold_default();
  if (m_threshold == 0 && viewParams.hasOccupancy) {
    m_threshold = 1; // Handle invalid depth for source views, transport views and viewports
  }
}

inline OccupancyTransform::OccupancyTransform(const ViewParams &viewParams,
                                              const PatchParams &patchParams) {
  m_threshold = patchParams.pduDepthOccMapThreshold() ? *patchParams.pduDepthOccMapThreshold()
                                           : viewParams.dq.dq_depth_occ_map_threshold_default();
  if (m_threshold == 0 && viewParams.hasOccupancy) {
    m_threshold = 1; // Handle invalid depth for source views, transport views and viewports
  }
}

inline auto OccupancyTransform::occupant(uint16_t x) const -> bool { return x >= m_threshold; }

template <unsigned bits>
DepthTransform<bits>::DepthTransform(const DepthQuantization &dq)
    : m_normDispLow{dq.dq_norm_disp_low()}, m_normDispHigh{dq.dq_norm_disp_high()} {}

template <unsigned bits>
DepthTransform<bits>::DepthTransform(const DepthQuantization &dq, const PatchParams &patchParams)
    : DepthTransform{dq} {
  if (patchParams.pduDepthStart()) {
    m_depthStart = *patchParams.pduDepthStart();
  }
  // TODO(BK): Implement pduDepthEnd
}

template <unsigned bits> auto DepthTransform<bits>::expandNormDisp(uint16_t x) const -> float {
  const auto level = Common::expandValue<bits>(std::max(m_depthStart, x));
  return std::max(impl::minNormDisp, m_normDispLow + (m_normDispHigh - m_normDispLow) * level);
}

template <unsigned bits> auto DepthTransform<bits>::expandDepth(uint16_t x) const -> float {
  return 1.F / expandNormDisp(x);
}

template <unsigned bits>
auto DepthTransform<bits>::expandDepth(const Common::Mat<uint16_t> &matrix) const
    -> Common::Mat<float> {
  auto depth = Common::Mat<float>(matrix.sizes());
  std::transform(std::begin(matrix), std::end(matrix), std::begin(depth),
                 [this](uint16_t x) { return expandDepth(x); });
  return depth;
}

template <unsigned bits>
auto DepthTransform<bits>::expandDepth(const Common::Depth16Frame &frame) const
    -> Common::Mat<float> {
  static_assert(bits == 16);
  return expandDepth(frame.getPlane(0));
}

template <unsigned bits>
auto DepthTransform<bits>::expandDepth(const Common::Depth10Frame &frame) const
    -> Common::Mat<float> {
  static_assert(bits == 10);
  return expandDepth(frame.getPlane(0));
}

template <unsigned bits>
auto DepthTransform<bits>::quantizeNormDisp(float x, uint16_t minLevel) const -> uint16_t {
  if (x > 0.F) {
    const auto level = (x - m_normDispLow) / (m_normDispHigh - m_normDispLow);
    return std::max(minLevel, Common::quantizeValue<bits>(level));
  }
  return 0;
}

template <unsigned bits>
template <typename DepthFrame>
auto DepthTransform<bits>::quantizeNormDisp(const Common::Mat<float> &matrix,
                                            uint16_t minLevel) const -> DepthFrame {
  static_assert(bits == DepthFrame::getBitDepth());
  auto frame = DepthFrame{int(matrix.width()), int(matrix.height())};
  std::transform(std::begin(matrix), std::end(matrix), std::begin(frame.getPlane(0)),
                 [=](float x) { return quantizeNormDisp(x, minLevel); });
  return frame;
}
} // namespace TMIV::MivBitstream
