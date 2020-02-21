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
#define _TMIV_MIVBITSTREAM_DEPTHOCCUPANCYTRANSFORM_H_

#include <TMIV/MivBitstream/IvAccessUnitParams.h>
#include <TMIV/MivBitstream/IvSequenceParams.h>

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
  auto occupant(uint16_t x) const -> bool;

private:
  uint16_t m_threshold{};
};

// Extract the depth transform for the specified view [and patch]
template <unsigned bits> class DepthTransform {
public:
  // Constructor for per-view depth transform signalling (source)
  explicit DepthTransform(const DepthQuantization &dq);

  // Constructor for per-view depth transform signalling (codec)
  DepthTransform(const DepthQuantization &dq, const PatchParams &patch);

  // Expand a level to normalized disparity [m^-1]
  //
  // The level is assumed to be a depth level (instead of "non-occupied/invalid")
  auto expandNormDisp(uint16_t x) const -> float;

  // Expand a level to depth [m]
  //
  // The level is assumed to be a depth level (instead of "non-occupied/invalid")
  auto expandDepth(uint16_t x) const -> float;

  // Expand a matrix of levels to depth [m]
  //
  // See also expandDepth(uint16_t)
  auto expandDepth(const Common::Mat<uint16_t> &matrix) const -> Common::Mat<float>;

  // Expand a frame of levels to depth [m]
  //
  // See also expandDepth(uint16_t)
  auto expandDepth(const Common::Depth16Frame &frame) const -> Common::Mat<float>;
  auto expandDepth(const Common::Depth10Frame &frame) const -> Common::Mat<float>;

  // Quantize normalized disparity [m^-1] to a level
  //
  // Invalid depth values are set to zero
  // Valid depth values are clamped to minLevel
  auto quantizeNormDisp(float x, uint16_t minLevel) const -> uint16_t;

  // Quantize a matrix of normalized disparities [m^-1] to a Depth16Frame
  //
  // See also quantizeNormDisp(float, uint16_t)
  template <typename DepthFrame = Common::Depth16Frame>
  auto quantizeNormDisp(const Common::Mat<float> &matrix, uint16_t minLevel) const -> DepthFrame;

private:
  const float m_normDispLow;
  const float m_normDispHigh;
  uint16_t m_depthStart{};
};
} // namespace TMIV::MivBitstream

#include "DepthOccupancyTransform.hpp"

#endif
