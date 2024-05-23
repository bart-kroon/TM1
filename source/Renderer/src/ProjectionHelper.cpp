/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2024, ISO/IEC
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

#include <TMIV/Renderer/ProjectionHelper.h>

namespace TMIV::Renderer {
ProjectionHelper::ProjectionHelper(const MivBitstream::ViewParams &viewParams)
    : m_viewParams{viewParams}
    , m_projector{makeProjector(viewParams.ci)}
    , m_rotation{Common::floatCast, viewParams.pose.orientation} {}

auto ProjectionHelper::getViewingPosition() const -> Common::Vec3f {
  return m_viewParams.pose.position;
}

auto ProjectionHelper::getViewingDirection() const -> Common::Vec3f {
  return rotate(Common::Vec3f{1.F, 0.F, 0.F}, m_rotation);
}

auto ProjectionHelper::doProjection(const Common::Vec3f &P) const
    -> std::pair<Common::Vec2f, float> {
  Common::Vec3f Q = rotate(P - m_viewParams.pose.position, conj(m_rotation));
  auto imageVertexDescriptor = m_projector->projectVertex(SceneVertexDescriptor{Q, 0.F});
  return std::make_pair(imageVertexDescriptor.position, imageVertexDescriptor.depth);
}

auto ProjectionHelper::doUnprojection(const Common::Vec2f &p, float d) const -> Common::Vec3f {
  auto P = m_projector->unprojectVertex(p, d);
  return rotate(P, m_rotation) + m_viewParams.pose.position;
}

auto ProjectionHelper::isStrictlyInsideViewport(const Common::Vec2f &p) const -> bool {
  return 0.5F <= p.x() && p.x() <= (m_viewParams.ci.projectionPlaneSizeF().x() - 0.5F) &&
         0.5F <= p.y() && p.y() <= (m_viewParams.ci.projectionPlaneSizeF().y() - 0.5F);
}

auto ProjectionHelper::isInsideViewport(const Common::Vec2f &p) const -> bool {
  return -0.5F <= p.x() && p.x() <= (m_viewParams.ci.projectionPlaneSizeF().x() + 0.5F) &&
         -0.5F <= p.y() && p.y() <= (m_viewParams.ci.projectionPlaneSizeF().y() + 0.5F);
}

auto ProjectionHelper::isValidDepth(float d) const -> bool {
  static constexpr auto far = 999.999F;
  return (TMIV::Renderer::isValidDepth(d) && (m_viewParams.dq.dq_norm_disp_low() <= (1.F / d)) &&
          (d < far));
}

auto ProjectionHelper::getAngularResolution() const -> float {
  const auto &ci = m_viewParams.ci;

  switch (ci.ci_cam_type()) {
  case MivBitstream::CiCamType::equirectangular: {
    auto nbPixel = ci.projectionPlaneSizeF().x() * ci.projectionPlaneSizeF().y();
    float DT = Common::deg2rad(ci.ci_erp_phi_max() - ci.ci_erp_phi_min());
    float DS = std::sin(Common::deg2rad(ci.ci_erp_theta_max())) -
               std::sin(Common::deg2rad(ci.ci_erp_theta_min()));

    return nbPixel / (DS * DT);
  }
  case MivBitstream::CiCamType::perspective: {
    auto nbPixel = ci.projectionPlaneSizeF().x() * ci.projectionPlaneSizeF().y();
    const auto projectionFocalLength =
        (ci.ci_perspective_focal_hor() + ci.ci_perspective_focal_ver()) / 2.F;
    auto w = ci.projectionPlaneSizeF().x();
    auto h = ci.projectionPlaneSizeF().y();
    float omega =
        4.F * std::atan(nbPixel /
                        (2.F * projectionFocalLength *
                         std::sqrt(4.F * Common::sqr(projectionFocalLength) + (w * w + h * h))));

    return nbPixel / omega;
  }
  case MivBitstream::CiCamType::orthographic: {
    auto nbPixel = ci.projectionPlaneSizeF().x() * ci.projectionPlaneSizeF().y();
    return nbPixel / Common::twoPi<float>;
  }
  default:
    UNREACHABLE;
  }
}

ProjectionHelperList::ProjectionHelperList(const MivBitstream::ViewParamsList &viewParamsList) {
  for (const auto &viewParams : viewParamsList) {
    this->emplace_back(viewParams);
  }
}
} // namespace TMIV::Renderer
