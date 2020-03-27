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

namespace TMIV::Renderer {
template <MivBitstream::CiCamType camType>
ProjectionHelper<camType>::List::List(const MivBitstream::ViewParamsList &viewParamsList) {
  for (const auto &viewParams : viewParamsList) {
    this->emplace_back(viewParams);
  }
}

template <MivBitstream::CiCamType camType>
ProjectionHelper<camType>::ProjectionHelper(const MivBitstream::ViewParams &viewParams)
    : m_viewParams{viewParams}, m_engine{viewParams.ci}, m_rotation{viewParams.ce.rotation()} {}

template <MivBitstream::CiCamType camType>
auto ProjectionHelper<camType>::getViewingDirection() const -> Common::Vec3f {
  return rotate(Common::Vec3f{1.F, 0.F, 0.F}, m_rotation);
}

template <MivBitstream::CiCamType camType>
auto ProjectionHelper<camType>::changeFrame(const Common::Vec3f &P) const -> Common::Vec3f {
  return rotate(P - m_viewParams.ce.position(), conj(m_rotation));
}

template <MivBitstream::CiCamType camType>
auto ProjectionHelper<camType>::doProjection(const Common::Vec3f &P) const
    -> std::pair<Common::Vec2f, float> {
  Common::Vec3f Q = changeFrame(P);
  auto imageVertexDescriptor = m_engine.projectVertex(SceneVertexDescriptor{Q, 0.F});
  return std::make_pair(imageVertexDescriptor.position, imageVertexDescriptor.depth);
}

template <MivBitstream::CiCamType camType>
auto ProjectionHelper<camType>::doUnprojection(const Common::Vec2f &p, float d) const
    -> Common::Vec3f {
  auto P = m_engine.unprojectVertex(p, d);
  return rotate(P, m_rotation) + m_viewParams.ce.position();
}

template <MivBitstream::CiCamType camType>
auto ProjectionHelper<camType>::isStrictlyInsideViewport(const Common::Vec2f &p) const -> bool {
  return ((0.5F <= p.x()) && (p.x() <= (m_viewParams.ci.projectionPlaneSize().x() - 0.5F))) &&
         ((0.5F <= p.y()) && (p.y() <= (m_viewParams.ci.projectionPlaneSize().y() - 0.5F)));
}

template <MivBitstream::CiCamType camType>
auto ProjectionHelper<camType>::isInsideViewport(const Common::Vec2f &p) const -> bool {
  return ((-0.5F <= p.x()) && (p.x() <= (m_viewParams.ci.projectionPlaneSize().x() + 0.5F))) &&
         ((-0.5F <= p.y()) && (p.y() <= (m_viewParams.ci.projectionPlaneSize().y() + 0.5F)));
}

template <MivBitstream::CiCamType camType>
auto ProjectionHelper<camType>::isValidDepth(float d) const -> bool {
  static constexpr auto far = 999.999F;
  return (TMIV::Renderer::isValidDepth(d) && (m_viewParams.dq.dq_norm_disp_low() <= (1.F / d)) &&
          (d < far));
}

template <MivBitstream::CiCamType camType>
auto ProjectionHelper<camType>::getDepthRange() const -> Common::Vec2f {
  return {1.F / m_viewParams.dq.dq_norm_disp_high(), 1.F / m_viewParams.dq.dq_norm_disp_low()};
}

template <MivBitstream::CiCamType camType>
auto ProjectionHelper<camType>::getPointCloud(unsigned N) const -> PointCloud {
  PointCloud pointCloud;
  float step = 1.F / static_cast<float>(N - 1U);
  auto depthRange = getDepthRange();

  float x = 0.F;

  for (unsigned i = 0U; i < N; i++) {
    float y = 0.F;

    float px = x * static_cast<float>(m_viewParams.ci.projectionPlaneSize().x());

    for (unsigned j = 0U; j < N; j++) {
      float d = depthRange.x();

      float py = y * static_cast<float>(m_viewParams.ci.projectionPlaneSize().y());

      for (unsigned k = 0U; k < N; k++) {
        pointCloud.emplace_back(doUnprojection({px, py}, d));

        d += step * (depthRange.y() - depthRange.x());
      }

      y += step;
    }

    x += step;
  }

  return pointCloud;
}

template <MivBitstream::CiCamType camType>
auto getPointCloudList(const ProjectionHelperList<camType> &sourceHelperList, unsigned N)
    -> PointCloudList {
  PointCloudList pointCloudList;

  for (const auto &helper : sourceHelperList) {
    pointCloudList.emplace_back(helper.getPointCloud(N));
  }

  return pointCloudList;
}

template <MivBitstream::CiCamType camType>
auto getOverlapping(const ProjectionHelperList<camType> &sourceHelperList,
                    const PointCloudList &pointCloudList, std::size_t firstId, std::size_t secondId)
    -> float {
  std::size_t N = 0;

  const ProjectionHelper<camType> &secondHelper = sourceHelperList[secondId];
  const PointCloud &firstPointCloud = pointCloudList[firstId];

  for (const auto &P : firstPointCloud) {
    auto p = secondHelper.doProjection(P);

    if (isValidDepth(p.second) && secondHelper.isInsideViewport(p.first)) {
      N++;
    }
  }

  return static_cast<float>(N) / static_cast<float>(firstPointCloud.size());
}

template <MivBitstream::CiCamType camType>
static auto computeOverlappingMatrix(const ProjectionHelperList<camType> &sourceHelperList)
    -> Common::Mat<float> {
  auto pointCloudList = getPointCloudList<camType>(sourceHelperList, 16);
  std::size_t K = sourceHelperList.size();
  Common::Mat<float> overlappingMatrix({K, K});

  for (std::size_t i = 0; i < K; i++) {
    for (std::size_t j = 0; j < K; j++) {
      if (i != j) {
        overlappingMatrix(i, j) = getOverlapping<camType>(sourceHelperList, pointCloudList, i, j);
      } else {
        overlappingMatrix(i, j) = 1.F;
      }
    }
  }

  return overlappingMatrix;
}
} // namespace TMIV::Renderer
