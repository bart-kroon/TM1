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

#ifndef _TMIV_DEPTHOCCUPANCY_DEPTHOCCUPANCY_H_
#define _TMIV_DEPTHOCCUPANCY_DEPTHOCCUPANCY_H_

#include <TMIV/DepthOccupancy/IDepthOccupancy.h>

#include <TMIV/Common/Json.h>

namespace TMIV::DepthOccupancy {
class DepthOccupancy : public IDepthOccupancy {
public:
  // Initialize with specified depthOccMapThreshold
  //
  // When incoming view parameters have depthOccMapThreshold > 0, then the outgoing view parameters
  // will have the specified depthOccMapThreshold value.
  explicit DepthOccupancy(uint16_t depthOccMapThreshold);

  DepthOccupancy(const Common::Json & /*unused*/, const Common::Json & /*unused*/);
  DepthOccupancy(const DepthOccupancy &) = default;
  DepthOccupancy(DepthOccupancy &&) = default;
  DepthOccupancy &operator=(const DepthOccupancy &) = default;
  DepthOccupancy &operator=(DepthOccupancy &&) = default;
  ~DepthOccupancy() override = default;

  // No change when depthOccMapThreshold == 0 (no invalid depth)
  // Otherwise set depthOccMapThreshold and adjust normDispRange
  auto transformSequenceParams(Metadata::IvSequenceParams)
      -> const Metadata::IvSequenceParams & override;

  // depthOccupancyParamsPresentFlags = zeros
  auto transformAccessUnitParams(Metadata::IvAccessUnitParams)
      -> const Metadata::IvAccessUnitParams & override;

  // Transform depth bit depth and range
  auto transformAtlases(const Common::MVD16Frame &inAtlases) -> Common::MVD10Frame override;

private:
  uint16_t m_depthOccMapThreshold{};
  Metadata::IvSequenceParams m_inSequenceParams;
  Metadata::IvSequenceParams m_outSequenceParams;
  Metadata::IvAccessUnitParams m_accessUnitParams;
};
} // namespace TMIV::DepthOccupancy

#endif
