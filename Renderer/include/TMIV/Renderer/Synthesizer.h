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

#ifndef _TMIV_RENDERER_SYNTHESIZER_H_
#define _TMIV_RENDERER_SYNTHESIZER_H_

#include <TMIV/Renderer/ISynthesizer.h>

namespace TMIV::Renderer {
class Synthesizer : public ISynthesizer {
public:
  Synthesizer(const Common::Json & /*unused*/, const Common::Json & /*componentNode*/);
  Synthesizer(float rayAngleParam, float depthParam, float stretchingParam, float maxStretching);
  Synthesizer(const Synthesizer &) = delete;
  Synthesizer(Synthesizer &&) = default;
  Synthesizer &operator=(const Synthesizer &) = delete;
  Synthesizer &operator=(Synthesizer &&) = default;
  ~Synthesizer() override;

  auto renderFrame(const Common::MVD10Frame &atlas, const Common::PatchIdMapList &maps,
                   const Metadata::AtlasParamsVector &patches,
                   const Metadata::ViewParamsVector &viewParamsVector,
                   const Metadata::ViewParams &target) const
      -> Common::Texture444Depth16Frame override;

  auto renderFrame(const Common::MVD10Frame &frame,
                   const Metadata::ViewParamsVector &viewParamsVector,
                   const Metadata::ViewParams &target) const
      -> Common::Texture444Depth16Frame override;

  auto renderDepth(const Common::Mat<float> &frame, const Metadata::ViewParams &viewParams,
                   const Metadata::ViewParams &target) const -> Common::Mat<float> override;

private:
  class Impl;

  std::unique_ptr<Impl> m_impl;
};
} // namespace TMIV::Renderer

#endif
