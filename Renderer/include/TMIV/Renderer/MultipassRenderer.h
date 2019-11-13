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

#ifndef _TMIV_RENDERER_MULTIPASSRENDERER_H_
#define _TMIV_RENDERER_MULTIPASSRENDERER_H_

#include <TMIV/Renderer/IInpainter.h>
#include <TMIV/Renderer/IRenderer.h>
#include <TMIV/Renderer/ISynthesizer.h>
#include <TMIV/Renderer/IViewingSpaceController.h>

namespace TMIV::Renderer {
enum class MergeMode {
  inpaint = 0, // let the inpainter fill
  lowPass = 1, // fill from the low-pass synthesis results which are in the background
  highPass = 2 // fill from the high-pass synthesis results which are in the foreground
};

// Advanced multipass implementation of IRenderer
class MultipassRenderer : public IRenderer {
private:
  std::unique_ptr<ISynthesizer> m_synthesizer;
  std::unique_ptr<IInpainter> m_inpainter;
  std::unique_ptr<IViewingSpaceController> m_viewingSpaceController;
  std::size_t m_numberOfPasses{};
  std::vector<std::size_t> m_numberOfViewsPerPass;
  MergeMode m_mergeConflict = MergeMode::lowPass;

public:
  MultipassRenderer(const Common::Json & /*rootNode*/, const Common::Json & /*componentNode*/);
  MultipassRenderer(const MultipassRenderer &) = delete;
  MultipassRenderer(MultipassRenderer &&) = default;
  MultipassRenderer &operator=(const MultipassRenderer &) = delete;
  MultipassRenderer &operator=(MultipassRenderer &&) = default;
  ~MultipassRenderer() override = default;

  auto renderFrame(const Common::MVD10Frame &atlas, const Common::PatchIdMapList &maps,
                   const Metadata::IvSequenceParams &ivSequenceParams,
                   const Metadata::IvAccessUnitParams &ivAccessUnitParams,
                   const Metadata::ViewParams &target) const
      -> Common::Texture444Depth16Frame override;
};
} // namespace TMIV::Renderer

#endif
