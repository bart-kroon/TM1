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

#include <TMIV/Renderer/Front/MultipleFrameRenderer.h>

#include <TMIV/Common/Factory.h>
#include <TMIV/Common/LoggingStrategyFmt.h>
#include <TMIV/MivBitstream/Formatters.h>
#include <TMIV/Renderer/ICuller.h>
#include <TMIV/Renderer/IRenderer.h>

using namespace std::string_literals;

namespace TMIV::Renderer::Front {
class MultipleFrameRenderer::Impl {
public:
  Impl(const Common::Json &rootNode, const std::vector<std::string> &outputCameraNames,
       const std::vector<std::string> &outputPoseTraceNames, IO::Placeholders placeholders);

  void renderMultipleFrames(const MivBitstream::AccessUnit &frame,
                            const FrameMapping::const_iterator &first,
                            const FrameMapping::const_iterator &last) const;

  [[nodiscard]] auto isOptimizedForRestrictedGeometry() const -> bool {
    // NOTe(FT): added to handle the absence of renderer in the G3 anchor type
    if (m_renderer) {
      return m_renderer->isOptimizedForRestrictedGeometry();
    }
    return false;
  }

private:
  void renderFrame(MivBitstream::AccessUnit frame, int32_t outputFrameIndex,
                   const std::string &cameraName, bool isPoseTrace) const;

  const Common::Json &m_config;
  const std::vector<std::string> &m_outputCameraNames;
  const std::vector<std::string> &m_outputPoseTraceNames;
  IO::Placeholders m_placeholders;
  std::unique_ptr<Renderer::ICuller> m_culler;
  std::unique_ptr<Renderer::IRenderer> m_renderer;
};

MultipleFrameRenderer::MultipleFrameRenderer(const Common::Json &rootNode,
                                             const std::vector<std::string> &outputCameraNames,
                                             const std::vector<std::string> &outputPoseTraceNames,
                                             IO::Placeholders placeholders)
    : m_impl{new Impl{rootNode, outputCameraNames, outputPoseTraceNames, std::move(placeholders)}} {
}

MultipleFrameRenderer::~MultipleFrameRenderer() = default;

void MultipleFrameRenderer::renderMultipleFrames(const MivBitstream::AccessUnit &frame,
                                                 const FrameMapping::const_iterator &first,
                                                 const FrameMapping::const_iterator &last) const {
  m_impl->renderMultipleFrames(frame, first, last);
}

auto MultipleFrameRenderer::isOptimizedForRestrictedGeometry() const -> bool {
  return m_impl->isOptimizedForRestrictedGeometry();
}

MultipleFrameRenderer::Impl::Impl(const Common::Json &rootNode,
                                  const std::vector<std::string> &outputCameraNames,
                                  const std::vector<std::string> &outputPoseTraceNames,
                                  IO::Placeholders placeholders)
    : m_config{rootNode}
    , m_outputCameraNames{outputCameraNames}
    , m_outputPoseTraceNames{outputPoseTraceNames}
    , m_placeholders{std::move(placeholders)} {
  if (!m_outputCameraNames.empty() || !m_outputPoseTraceNames.empty()) {
    if (!rootNode.optional("RendererMethod")) {
      throw std::runtime_error("The configuration does not support rendering viewports");
    }
    m_culler = Common::create<ICuller>("Culler"s, rootNode, rootNode);
    m_renderer = Common::create<IRenderer>("Renderer"s, rootNode, rootNode);
  }
}

void MultipleFrameRenderer::Impl::renderMultipleFrames(
    const MivBitstream::AccessUnit &frame, const FrameMapping::const_iterator &first,
    const FrameMapping::const_iterator &last) const {
  for (auto i = first; i != last; ++i) {
    if (i->first == i->second) {
      for (const auto &name : m_outputCameraNames) {
        renderFrame(frame, i->second, name, false);
      }
    }
    for (const auto &name : m_outputPoseTraceNames) {
      renderFrame(frame, i->second, name, true);
    }
  }
}

void MultipleFrameRenderer::Impl::renderFrame(MivBitstream::AccessUnit frame,
                                              int32_t outputFrameIndex,
                                              const std::string &cameraName,
                                              bool isPoseTrace) const {
  Common::logInfo("Rendering input frame {} to output frame {} for target {} {}.", frame.frameIdx,
                  outputFrameIndex, isPoseTrace ? "pose trace" : "view", cameraName);

  const auto viewportParams =
      IO::loadViewportMetadata(m_config, m_placeholders, outputFrameIndex, cameraName, isPoseTrace);

  m_culler->inplaceFilterBlockToPatchMaps(frame, viewportParams.viewParams);

  const auto viewport = m_renderer->renderFrame(frame, viewportParams);
  IO::saveViewport(m_config, m_placeholders, outputFrameIndex, cameraName,
                   {yuv420(viewport.texture), yuv420(viewport.geometry)});
}
} // namespace TMIV::Renderer::Front
