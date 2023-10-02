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

#ifndef TMIV_RENDERER_PROJECTIONHELPER_H
#define TMIV_RENDERER_PROJECTIONHELPER_H

#include "Projector.h"

namespace TMIV::Renderer {
constexpr auto isValidDepth(float d) -> bool { return 0.F < d; }

class ProjectionHelper {
private:
  MivBitstream::ViewParams m_viewParams;
  std::unique_ptr<IProjector> m_projector;
  Common::QuatF m_rotation{Common::neutralOrientationF};

public:
  explicit ProjectionHelper(const MivBitstream::ViewParams &viewParams);

  ProjectionHelper(const ProjectionHelper &) = delete;
  ProjectionHelper(ProjectionHelper &&) = default;
  auto operator=(const ProjectionHelper &) -> ProjectionHelper & = delete;
  auto operator=(ProjectionHelper &&) -> ProjectionHelper & = default;
  ~ProjectionHelper() = default;

  [[nodiscard]] auto getViewParams() const -> const auto & { return m_viewParams; }
  [[nodiscard]] auto getViewingPosition() const -> Common::Vec3f;
  [[nodiscard]] auto getViewingDirection() const -> Common::Vec3f;
  [[nodiscard]] auto doProjection(const Common::Vec3f &P) const -> std::pair<Common::Vec2f, float>;
  [[nodiscard]] auto doUnprojection(const Common::Vec2f &p, float d) const -> Common::Vec3f;
  [[nodiscard]] auto isStrictlyInsideViewport(const Common::Vec2f &p) const -> bool;
  [[nodiscard]] auto isInsideViewport(const Common::Vec2f &p) const -> bool;
  [[nodiscard]] auto isValidDepth(float d) const -> bool;
  [[nodiscard]] auto getAngularResolution() const -> float;
};

class ProjectionHelperList : public std::vector<ProjectionHelper> {
public:
  ProjectionHelperList(const MivBitstream::ViewParamsList &viewParamsList);
};
} // namespace TMIV::Renderer

#endif
