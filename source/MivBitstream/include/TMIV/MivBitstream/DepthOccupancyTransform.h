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

#ifndef TMIV_MIVBITSTREAM_DEPTHOCCUPANCYTRANSFORM_H
#define TMIV_MIVBITSTREAM_DEPTHOCCUPANCYTRANSFORM_H

#include "PatchParamsList.h"
#include "ViewParamsList.h"

#include <TMIV/Common/Frame.h>

namespace TMIV::MivBitstream {
// Extract the occupancy transform for the specified view [and patch]
class OccupancyTransform {
public:
  // Constructor for per-view occupancy threshold signalling (source)
  explicit OccupancyTransform(const ViewParams &viewParams);

  // Constructor for per-patch occupancy threshold signalling (codec)
  OccupancyTransform(const ViewParams &viewParams, const PatchParams &patch);

  // Does x indicate "occupied/valid"?
  [[nodiscard]] auto occupant(Common::SampleValue x) const -> bool;

private:
  Common::SampleValue m_threshold{};
};

// Extract the depth transform for the specified view [and patch]
class DepthTransform {
public:
  // Constructor for per-view depth transform signalling (source)
  explicit DepthTransform(const DepthQuantization &dq, uint32_t bitDepth);

  // Constructor for per-patch depth transform signalling (codec)
  DepthTransform(const DepthQuantization &dq, const PatchParams &patch, uint32_t bitDepth);

  // Expand a level to normalized disparity [m^-1]
  //
  // The level is assumed to be a depth level (instead of "non-occupied/invalid")
  [[nodiscard]] auto expandNormDisp(Common::SampleValue x) const -> float;

  // Expand a level to depth [m]
  //
  // The level is assumed to be a depth level (instead of "non-occupied/invalid")
  [[nodiscard]] auto expandDepth(Common::SampleValue x) const -> float;

  // Expand a matrix of levels to depth [m]
  //
  // See also expandDepth(uint16_t)
  [[nodiscard]] auto expandDepth(const Common::Mat<> &matrix) const -> Common::Mat<float>;

  // Expand a frame of levels to depth [m]
  //
  // See also expandDepth(uint16_t)
  [[nodiscard]] auto expandDepth(const Common::Frame<> &frame) const -> Common::Mat<float>;

  // Quantize normalized disparity [m^-1] to a level
  //
  // Invalid depth values are set to zero
  // Valid depth values are clamped to minLevel
  [[nodiscard]] auto quantizeNormDisp(float x, Common::SampleValue minLevel) const
      -> Common::SampleValue;

  // Quantize a matrix of normalized disparities [m^-1] to a frame
  //
  // See also quantizeNormDisp(float, uint16_t)
  [[nodiscard]] auto quantizeNormDisp(const Common::Mat<float> &matrix,
                                      Common::SampleValue minLevel) const -> Common::Frame<>;

  // Implementation-defined minimum normalized disparity [m^-1]
  //
  // This value is a positive value (less than infinite depth) to simplify reprojection
  //
  // For a practical application this can be a fixed value (e.g. (1 km)^-1 but the test model does
  // not require lengths to be provided as meters and we cannot assume that 0.001 is low enough, nor
  // do we want the value to be much too low because that will reduce numerical accuracy of point
  // reprojections.
  [[nodiscard]] auto minNormDisp() const -> float;

private:
  float m_normDispLow{};
  float m_normDispHigh{};
  float m_minNormDisp{};
  uint32_t m_bitDepth{};
  Common::SampleValue m_maxSampleValue{};
  Common::SampleValue m_atlasPatch3dOffsetD{};
  Common::SampleValue m_atlasPatch3dRangeD{std::numeric_limits<Common::SampleValue>::max()};

  uint8_t m_viewPivotCount{};
  uint8_t m_quantizationLaw{};
  std::vector<float> m_normDispMap;
  float m_normDispInterval{};
  float m_normDispMax{};
};

} // namespace TMIV::MivBitstream

#include "DepthOccupancyTransform.hpp"

#endif
