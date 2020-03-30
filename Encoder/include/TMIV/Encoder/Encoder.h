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

#ifndef _TMIV_ENCODER_ENCODER_H_
#define _TMIV_ENCODER_ENCODER_H_

#include <TMIV/Encoder/IEncoder.h>

#include <TMIV/AtlasConstructor/IAtlasConstructor.h>
#include <TMIV/Common/Json.h>
#include <TMIV/DepthOccupancy/IDepthOccupancy.h>
#include <TMIV/Encoder/GeometryDownscaler.h>
#include <TMIV/ViewOptimizer/IViewOptimizer.h>

namespace TMIV::Encoder {
class Encoder : public IEncoder {
public:
  Encoder(const Common::Json & /*rootNode*/, const Common::Json & /*componentNode*/);
  Encoder(const Encoder &) = delete;
  Encoder(Encoder &&) = default;
  auto operator=(const Encoder &) -> Encoder & = delete;
  auto operator=(Encoder &&) -> Encoder & = default;
  ~Encoder() override = default;

  auto prepareSequence(MivBitstream::IvSequenceParams ivSequenceParams)
      -> const MivBitstream::IvSequenceParams & override;
  void prepareAccessUnit(MivBitstream::IvAccessUnitParams ivAccessUnitParams) override;
  void pushFrame(Common::MVD16Frame views) override;
  auto completeAccessUnit() -> const MivBitstream::IvAccessUnitParams & override;
  auto popAtlas() -> Common::MVD10Frame override;
  [[nodiscard]] auto maxLumaSamplesPerFrame() const -> std::size_t override;

private:
  std::unique_ptr<ViewOptimizer::IViewOptimizer> m_viewOptimizer;
  std::unique_ptr<AtlasConstructor::IAtlasConstructor> m_atlasConstructor;
  std::unique_ptr<DepthOccupancy::IDepthOccupancy> m_depthOccupancy;
  GeometryDownscaler m_geometryDownscaler;
};
} // namespace TMIV::Encoder

#endif
