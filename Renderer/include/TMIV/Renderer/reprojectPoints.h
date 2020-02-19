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
#include <TMIV/MivBitstream/IvSequenceParams.h>
#include <TMIV/Renderer/Engine.h>

namespace TMIV::Renderer {
// Create a grid of positions indicating the center of each of the pixels
auto imagePositions(const MivBitstream::ViewParams &viewParams) -> Common::Mat<Common::Vec2f>;

// OMAF Referential: x forward, y left, z up
// Image plane: u right, v down

// Unproject points: From image positions to world positions (with the camera as
// reference frame)
auto unprojectPoints(const MivBitstream::ViewParams &viewParams,
                     const Common::Mat<Common::Vec2f> &positions, const Common::Mat<float> &depth)
    -> Common::Mat<Common::Vec3f>;

// Change the reference frame from one to another camera (merging extrinsic
// parameters)
auto changeReferenceFrame(const MivBitstream::ViewParams &viewParams,
                          const MivBitstream::ViewParams &target,
                          const Common::Mat<Common::Vec3f> &points) -> Common::Mat<Common::Vec3f>;

// Project points: From world positions (with the camera as reference frame)
// to image positions
auto projectPoints(const MivBitstream::ViewParams &viewParams,
                   const Common::Mat<Common::Vec3f> &points)
    -> std::pair<Common::Mat<Common::Vec2f>, Common::Mat<float>>;

// Reproject points by combining above three steps:
//  1) Unproject to world points in the reference frame of the first camera
//  2) Change the reference frame from the first to the second camera
//  3) Project to image points
auto reprojectPoints(const MivBitstream::ViewParams &viewParams,
                     const MivBitstream::ViewParams &target,
                     const Common::Mat<Common::Vec2f> &positions, const Common::Mat<float> &depth)
    -> std::pair<Common::Mat<Common::Vec2f>, Common::Mat<float>>;

// Calculate ray angles between input and output camera. Units are radians.
//
// The points should be in the target frame of reference.
auto calculateRayAngles(const MivBitstream::ViewParams &viewParams,
                        const MivBitstream::ViewParams &target,
                        const Common::Mat<Common::Vec3f> &points) -> Common::Mat<float>;

// Return (R, T) such that x -> Rx + t changes reference frame from the source
// camera to the target camera
auto affineParameters(const MivBitstream::ViewParams &viewParams,
                      const MivBitstream::ViewParams &target)
    -> std::pair<Common::Mat3x3f, Common::Vec3f>;

// Unproject a pixel from a source frame to scene coordinates in the reference
// frame of the target camera.
//
// This method is less efficient because of the switch on projection type, but
// suitable for rendering directly from an atlas.
auto unprojectVertex(Common::Vec2f position, float depth,
                     const MivBitstream::ViewParams &viewParams) -> Common::Vec3f;

// Project point: From world position (with the camera as reference frame)
// to image position
//
// This method is less efficient because of the switch on projection type, but
// suitable for rendering directly from an atlas.
auto projectVertex(const Common::Vec3f &position, const MivBitstream::ViewParams &viewParams)
    -> std::pair<Common::Vec2f, float>;

inline bool isValidDepth(float d) { return (0.F < d); }

using PointCloud = std::vector<Common::Vec3f>;
using PointCloudList = std::vector<PointCloud>;

template <MivBitstream::CiCamType camType> class ProjectionHelper {
public:
  class List : public std::vector<ProjectionHelper> {
  public:
    List(const MivBitstream::ViewParamsList &viewParamsList);
    List(const List &) = default;
    List(List &&) = default;
    auto operator=(const List &) -> List & = default;
    auto operator=(List &&) -> List & = default;
  };

private:
  const MivBitstream::ViewParams &m_viewParams;
  Engine<camType> m_engine;
  Common::Mat3x3f m_rotationMatrix;

public:
  ProjectionHelper(const MivBitstream::ViewParams &viewParams);
  ProjectionHelper(const ProjectionHelper &) = default;
  ProjectionHelper(ProjectionHelper &&) = default;
  auto operator=(const ProjectionHelper &) -> ProjectionHelper & = default;
  auto operator=(ProjectionHelper &&) -> ProjectionHelper & = default;
  auto getViewParams() const -> const MivBitstream::ViewParams & { return m_viewParams; }
  auto getViewingPosition() const -> Common::Vec3f { return m_viewParams.ce.position(); }
  auto getViewingDirection() const -> Common::Vec3f;
  auto changeFrame(const Common::Vec3f &P) const -> Common::Vec3f;
  auto doProjection(const Common::Vec3f &P) const -> std::pair<Common::Vec2f, float>;
  auto doUnprojection(const Common::Vec2f &p, float d) const -> Common::Vec3f;
  auto isStrictlyInsideViewport(const Common::Vec2f &p) const -> bool;
  auto isInsideViewport(const Common::Vec2f &p) const -> bool;
  bool isValidDepth(float d) const;
  auto getAngularResolution() const -> float;
  auto getDepthRange() const -> Common::Vec2f;
  auto getRadialRange() const -> Common::Vec2f;
  auto getPointCloud(unsigned N = 8) const -> PointCloud;
};

template <MivBitstream::CiCamType camType>
auto getPointCloudList(const typename ProjectionHelper<camType>::List &sourceHelperList,
                       unsigned N = 16) -> PointCloudList;

template <MivBitstream::CiCamType camType>
auto getOverlapping(const typename ProjectionHelper<camType>::List &sourceHelperList,
                    const PointCloudList &pointCloudList, std::size_t firstId, std::size_t secondId)
    -> float;

template <MivBitstream::CiCamType camType>
static auto
computeOverlappingMatrix(const typename ProjectionHelper<camType>::List &sourceHelperList)
    -> Common::Mat<float>;

} // namespace TMIV::Renderer

#include "reprojectPoints.hpp"

#endif
