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

#include <TMIV/Common/Factory.h>
#include <TMIV/Renderer/Renderer.h>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Renderer {
Renderer::Renderer(const Common::Json &rootNode,
                   const Common::Json &componentNode)
    : m_synthesizer{Factory<ISynthesizer>::getInstance().create(
          "Synthesizer", rootNode, componentNode)},
      m_inpainter{Factory<IInpainter>::getInstance().create(
          "Inpainter", rootNode, componentNode)} {}

Common::Texture444Depth10Frame
Renderer::renderFrame(const Common::MVD10Frame &atlas,
                      const Common::PatchIdMapList &maps,
                      const Metadata::AtlasParametersList &patches,
                      const Metadata::CameraParametersList &cameras,
                      const Metadata::CameraParameters &target) const {
  auto viewport =
      m_synthesizer->renderFrame(atlas, maps, patches, cameras, target);
  m_inpainter->inplaceInpaint(viewport, target);
  return viewport;
}

Common::Texture444Depth16Frame
Renderer::renderFrame(const Common::MVD16Frame &frame,
                      const Metadata::CameraParametersList &cameras,
                      const Metadata::CameraParameters &target) const {
  auto viewport = m_synthesizer->renderFrame(frame, cameras, target);
  m_inpainter->inplaceInpaint(viewport, target);
  return viewport;
}
} // namespace TMIV::Renderer
