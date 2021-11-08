/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ISO/IEC
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

#ifndef TMIV_RENDERER_PUSHPULL_H
#define TMIV_RENDERER_PUSHPULL_H

#include <TMIV/Common/Frame.h>

namespace TMIV::Renderer {
template <typename Matrix> struct MatrixProxy {
  Matrix matrix;
  int32_t width;
  int32_t height;
};

template <typename Matrix> MatrixProxy(Matrix &&, int32_t, int32_t) -> MatrixProxy<Matrix>;

class PushPull {
public:
  template <typename PushFilter, typename PullFilter>
  auto filter(const Common::RendererFrame &frame, PushFilter &&pushFilter, PullFilter &&pullFilter)
      -> const Common::RendererFrame &;

  auto numLayers() const noexcept;

  auto layer(size_t i) const noexcept -> const Common::RendererFrame &;

  // View a texture+depth frame as a matrix of tuple values
  static auto yuvd(const Common::RendererFrame &frame);

  // View a texture+depth frame as a matrix of tuple references
  static auto yuvd(Common::RendererFrame &frame);

  template <typename PushFilter>
  static void inplacePushFrame(const Common::RendererFrame &in, Common::RendererFrame &out,
                               PushFilter &&filter);

  template <typename PullFilter>
  static void inplacePullFrame(const Common::RendererFrame &in, Common::RendererFrame &out,
                               PullFilter &&filter) noexcept;

  template <typename InMatrixProxy, typename OutMatrixProxy, typename PushFilter>
  static void inplacePush(const InMatrixProxy &&in, OutMatrixProxy &&out,
                          PushFilter &&filter) noexcept;

  template <typename InMatrixProxy, typename OutMatrixProxy, typename PullFilter>
  static void inplacePull(const InMatrixProxy &&in, OutMatrixProxy &&out,
                          PullFilter &&filter) noexcept;

private:
  std::vector<Common::RendererFrame> m_pyramid;
};
} // namespace TMIV::Renderer

#include "PushPull.hpp"

#endif
