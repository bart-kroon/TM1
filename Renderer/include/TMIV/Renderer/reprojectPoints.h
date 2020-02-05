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

#ifndef _TMIV_RENDERER_REPROJECTPOINTS_H_
#define _TMIV_RENDERER_REPROJECTPOINTS_H_

#include <TMIV/Common/LinAlg.h>
#include <TMIV/Common/Transformation.h>
#include <TMIV/Metadata/IvSequenceParams.h>
#include <TMIV/Renderer/Engine.h>

namespace TMIV::Renderer {
// Create a grid of positions indicating the center of each of the pixels
auto imagePositions(const Metadata::ViewParams &viewParams) -> Common::Mat<Common::Vec2f>;

// OMAF Referential: x forward, y left, z up
// Image plane: u right, v down

// Unproject points: From image positions to world positions (with the camera as
// reference frame)
auto unprojectPoints(const Metadata::ViewParams &viewParams,
                     const Common::Mat<Common::Vec2f> &positions, const Common::Mat<float> &depth)
    -> Common::Mat<Common::Vec3f>;

// Change the reference frame from one to another camera (merging extrinsic
// parameters)
auto changeReferenceFrame(const Metadata::ViewParams &viewParams,
                          const Metadata::ViewParams &target,
                          const Common::Mat<Common::Vec3f> &points) -> Common::Mat<Common::Vec3f>;

// Project points: From world positions (with the camera as reference frame)
// to image positions
auto projectPoints(const Metadata::ViewParams &viewParams, const Common::Mat<Common::Vec3f> &points)
    -> std::pair<Common::Mat<Common::Vec2f>, Common::Mat<float>>;

// Reproject points by combining above three steps:
//  1) Unproject to world points in the reference frame of the first camera
//  2) Change the reference frame from the first to the second camera
//  3) Project to image points
auto reprojectPoints(const Metadata::ViewParams &viewParams, const Metadata::ViewParams &target,
                     const Common::Mat<Common::Vec2f> &positions, const Common::Mat<float> &depth)
    -> std::pair<Common::Mat<Common::Vec2f>, Common::Mat<float>>;

// Calculate ray angles between input and output camera. Units are radians.
//
// The points should be in the target frame of reference.
auto calculateRayAngles(const Metadata::ViewParams &viewParams, const Metadata::ViewParams &target,
                        const Common::Mat<Common::Vec3f> &points) -> Common::Mat<float>;

// Return (R, T) such that x -> Rx + t changes reference frame from the source
// camera to the target camera
auto affineParameters(const Metadata::ViewParams &viewParams, const Metadata::ViewParams &target)
    -> std::pair<Common::Mat3x3f, Common::Vec3f>;

// Unproject a pixel from a source frame to scene coordinates in the reference
// frame of the target camera.
//
// This method is less efficient because of the switch on projection type, but
// suitable for rendering directly from an atlas.
auto unprojectVertex(Common::Vec2f position, float depth, const Metadata::ViewParams &viewParams)
    -> Common::Vec3f;

// Project point: From world position (with the camera as reference frame)
// to image position
//
// This method is less efficient because of the switch on projection type, but
// suitable for rendering directly from an atlas.
auto projectVertex(const Common::Vec3f &position, const Metadata::ViewParams &viewParams)
    -> std::pair<Common::Vec2f, float>;

inline bool isValidDepth(float z) { return ((0.F < z) && std::isfinite(z)); }

using PointCloud = std::vector<Common::Vec3f>;
using PointCloudList = std::vector<PointCloud>;

template <typename Projection> class ProjectionHelper {
public:
  class List : public std::vector<ProjectionHelper> {
  public:
    List(const Metadata::ViewParamsList &viewParamsList) {
      for (const auto &viewParams : viewParamsList) {
        this->emplace_back(viewParams);
      }
    }
    List(const List &) = default;
    List(List &&) = default;
    auto operator=(const List &) -> List & = default;
    auto operator=(List &&) -> List & = default;
  };

private:
  const Metadata::ViewParams &m_viewParams;
  Engine<Projection> m_engine;
  Common::Mat3x3f m_rotationMatrix;

public:
  ProjectionHelper(const Metadata::ViewParams &viewParams)
      : m_viewParams{viewParams}, m_engine{viewParams},
        m_rotationMatrix{
            Common::EulerAnglesToRotationMatrix(Common::EulerAngles{viewParams.rotation})} {}
  ProjectionHelper(const ProjectionHelper &) = default;
  ProjectionHelper(ProjectionHelper &&) = default;
  auto operator=(const ProjectionHelper &) -> ProjectionHelper & = default;
  auto operator=(ProjectionHelper &&) -> ProjectionHelper & = default;
  auto getViewParams() const -> const Metadata::ViewParams & { return m_viewParams; }
  auto getViewingPosition() const -> const Common::Vec3f & { return m_viewParams.position; }
  auto getViewingDirection() const -> Common::Vec3f {
    return m_rotationMatrix * Common::Vec3f{1.F, 0.F, 0.F};
  }
  auto changeFrame(const Common::Vec3f &P) const -> Common::Vec3f {
    return transpose(m_rotationMatrix) * (P - m_viewParams.position);
  }
  auto doProjection(const Common::Vec3f &P) const -> std::pair<Common::Vec2f, float> {
    Common::Vec3f Q = transpose(m_rotationMatrix) * (P - m_viewParams.position);
    auto imageVertexDescriptor = m_engine.projectVertex(SceneVertexDescriptor{Q, 0.F});
    return std::make_pair(imageVertexDescriptor.position, imageVertexDescriptor.depth);
  }
  auto doUnprojection(const Common::Vec2f &p, float z) const -> Common::Vec3f {
    auto P = m_engine.unprojectVertex(p, z);
    return (m_rotationMatrix * P + m_viewParams.position);
  }
  auto isInsideViewport(const Common::Vec2f &p) const -> bool {
    return ((-0.5F <= p.x()) && (p.x() <= (m_viewParams.size.x() + 0.5F))) &&
           ((-0.5F <= p.y()) && (p.y() <= (m_viewParams.size.y() + 0.5F)));
  }
  bool isValidDepth(float z) const {
    static constexpr auto far = 999.999F;
    return (TMIV::Renderer::isValidDepth(z) && (m_viewParams.normDispRange[0] <= (1.F / z)) &&
            (z < far));
  }
  auto getAngularResolution() const -> float;
  auto getDepthRange() const -> Common::Vec2f {
    return {1.F / m_viewParams.normDispRange[1], 1.F / m_viewParams.normDispRange[0]};
  }
  auto getRadialRange() const -> Common::Vec2f;
  auto getPointCloud(unsigned N = 8) const -> PointCloud {
    PointCloud pointCloud;
    float step = 1.F / static_cast<float>(N - 1U);
    auto depthRange = getDepthRange();

    float x = 0.F;

    for (unsigned i = 0U; i < N; i++) {
      float y = 0.F;

      float px = x * static_cast<float>(m_viewParams.size[0]);

      for (unsigned j = 0U; j < N; j++) {
        float z = depthRange.x();

        float py = y * static_cast<float>(m_viewParams.size[1]);

        for (unsigned k = 0U; k < N; k++) {
          pointCloud.emplace_back(doUnprojection({px, py}, z));

          z += step * (depthRange.y() - depthRange.x());
        }

        y += step;
      }

      x += step;
    }

    return pointCloud;
  }
};

template <typename SourceProjectionType>
auto getPointCloudList(
    const typename ProjectionHelper<SourceProjectionType>::List &sourceHelperList, unsigned N = 16)
    -> PointCloudList {
  PointCloudList pointCloudList;

  for (const auto &helper : sourceHelperList) {
    pointCloudList.emplace_back(helper.getPointCloud(N));
  }

  return pointCloudList;
}

template <typename ProjectionType>
auto getOverlapping(const typename ProjectionHelper<ProjectionType>::List &sourceHelperList,
                    const PointCloudList &pointCloudList, std::size_t firstId, std::size_t secondId)
    -> float {
  std::size_t N = 0;

  const ProjectionHelper<ProjectionType> &secondHelper = sourceHelperList[secondId];
  const PointCloud &firstPointCloud = pointCloudList[firstId];

  for (const auto &P : firstPointCloud) {

    auto p = secondHelper.doProjection(P);

    if (isValidDepth(p.second) && secondHelper.isInsideViewport(p.first)) {
      N++;
    }
  }

  return static_cast<float>(N) / static_cast<float>(firstPointCloud.size());
}

template <typename ProjectionType>
static auto
computeOverlappingMatrix(const typename ProjectionHelper<ProjectionType>::List &sourceHelperList)
    -> Common::Mat<float> {

  auto pointCloudList = getPointCloudList<ProjectionType>(sourceHelperList, 16);
  std::size_t K = sourceHelperList.size();
  Common::Mat<float> overlappingMatrix({K, K});

  for (std::size_t i = 0; i < K; i++) {
    for (std::size_t j = 0; j < K; j++) {

      if (i != j) {
        overlappingMatrix(i, j) =
            getOverlapping<ProjectionType>(sourceHelperList, pointCloudList, i, j);
      } else {
        overlappingMatrix(i, j) = 1.F;
      }
    }
  }

  return overlappingMatrix;
}

} // namespace TMIV::Renderer

#endif
