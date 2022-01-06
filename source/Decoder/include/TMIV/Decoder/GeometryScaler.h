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

#ifndef TMIV_DECODER_GEOMETRYSCALER_H
#define TMIV_DECODER_GEOMETRYSCALER_H

#include <TMIV/Common/Frame.h>
#include <TMIV/Common/Json.h>
#include <TMIV/MivBitstream/AccessUnit.h>

namespace TMIV::Decoder {
class GeometryScaler {
public:
  // The node is optional because the decoder does not always have to render, for instance when
  // producing a decoder output log for conformance testing.
  explicit GeometryScaler(const Common::Json &optionalNode);

  [[nodiscard]] auto
  scale(const MivBitstream::AtlasAccessUnit &atlas, const Common::Frame<> &geoFrameNF,
        const std::optional<MivBitstream::GeometryUpscalingParameters> &gup) const
      -> Common::Frame<>;

  [[nodiscard]] static auto scale(const MivBitstream::AtlasAccessUnit &atlas,
                                  const Common::Frame<> &geoFrameNF,
                                  const MivBitstream::GeometryUpscalingParameters &gup)
      -> Common::Frame<>;

private:
  MivBitstream::GeometryUpscalingParameters m_defaultGup =
      MivBitstream::GeometryUpscalingParameters{}
          .gup_type(MivBitstream::GupType::HVR)
          .gup_erode_threshold(Common::Half{0.5F})
          .gup_delta_threshold(10)
          .gup_max_curvature(5);
};
} // namespace TMIV::Decoder

#endif