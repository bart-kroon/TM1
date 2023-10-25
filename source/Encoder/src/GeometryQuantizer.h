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

#ifndef TMIV_ENCODER_GEOMETRYQUANTIZER_H
#define TMIV_ENCODER_GEOMETRYQUANTIZER_H

#include <TMIV/Common/Distribution.h>
#include <TMIV/Common/Frame.h>
#include <TMIV/MivBitstream/EncoderParams.h>

#include "Configuration.h"

namespace TMIV::Encoder {
using MivBitstream::EncoderParams;

// Measure the distribution of geometry samples (in scene units) over views and patches
struct GeometryDistributions {
  std::vector<Common::Distribution> views;
  std::vector<Common::Distribution> patches;

  void report(const EncoderParams &params) const;

  static auto measure(const std::vector<Common::DeepFrameList> &videoFrameBuffer,
                      const EncoderParams &params) -> GeometryDistributions;
};

class GeometryQuantizer {
public:
  explicit GeometryQuantizer(const Configuration &config) : m_config{config} {}

  [[nodiscard]] auto transformParams(const GeometryDistributions &distributions,
                                     EncoderParams params) const -> EncoderParams;
  [[nodiscard]] static auto transformAtlases(const EncoderParams &inParams,
                                             const EncoderParams &outParams,
                                             const Common::DeepFrameList &inAtlases)
      -> Common::V3cFrameList;

private:
  void determineDepthRange(const GeometryDistributions &distributions, EncoderParams &params) const;
  void setDepthOccThreshold(EncoderParams &params) const;

  Configuration m_config;
};

[[nodiscard]] auto
transformGeometryQuantizationParams(const Configuration &config,
                                    const std::vector<Common::DeepFrameList> &videoFrameBuffer,
                                    const EncoderParams &params) -> EncoderParams;
} // namespace TMIV::Encoder

#endif
