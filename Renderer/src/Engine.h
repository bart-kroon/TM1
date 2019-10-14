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

#ifndef _TMIV_RENDERER_ENGINE_H_
#define _TMIV_RENDERER_ENGINE_H_

#include <TMIV/Common/LinAlg.h>
#include <TMIV/Metadata/IvSequenceParams.h>

namespace TMIV::Renderer {
struct SceneVertexDescriptor {
  Common::Vec3f position; // m, scene point in target reference frame
  float rayAngle{};       // rad, ray angle from: cos a = <v, w>/|v||w|
};

using SceneVertexDescriptorList = std::vector<SceneVertexDescriptor>;

struct TriangleDescriptor {
  std::array<int, 3> indices; // indices into vertex lists
  float area;                 // px�, area before unprojection
};

using TriangleDescriptorList = std::vector<TriangleDescriptor>;

struct ImageVertexDescriptor {
  Common::Vec2f position; // px, position in image (x right, y down)
  float depth{};          // m, depth as defined in the target projection
  float rayAngle{};       // rad, ray angle from: cos a = <v, w>/|v||w|
};

using ImageVertexDescriptorList = std::vector<ImageVertexDescriptor>;

// Return (R, T) such that x -> Rx + t changes reference frame from the source
// camera to the target camera
auto affineParameters(const Metadata::CameraParameters &camera,
                      const Metadata::CameraParameters &target)
    -> std::pair<Common::Mat3x3f, Common::Vec3f>;

// The rendering engine is the part that is specalized per projection type
template <typename Projection> struct Engine {};
} // namespace TMIV::Renderer

#include "Engine_ERP.hpp"
#include "Engine_Perspective.hpp"

namespace TMIV::Renderer {
// Unproject from a source frame to scene coordinates in the reference frame of
// the target camera, generating lists of vertices, triangles and attributes.
//
// This method is designed to allow for specialization per source camera
// projection.
template <typename Engine, typename... T>
auto unproject(const Engine &engine, const Common::Mat<float> &depth,
               const Metadata::CameraParameters &target, const Common::Mat<T> &... matrices) {
  return std::tuple{engine.makeSceneVertexDescriptorList(depth, target),
                    engine.makeTriangleDescriptorList(),
                    std::tuple{engine.makeVertexAttributeList(matrices)...}};
}

// Unproject from a source frame to scene coordinates in the reference frame of
// the target camera, generating lists of vertices, triangles and attributes.
//
// This method is designed to allow for specialization per source camera
// projection.
template <typename... T>
auto unproject(const Common::Mat<float> &depth, const Metadata::CameraParameters &camera,
               const Metadata::CameraParameters &target, const Common::Mat<T> &... matrices) {
  return visit(
      [&](auto const &x) {
        Engine<decay_t<decltype(x)>> engine{camera};
        return unproject(engine, depth, target, matrices...);
      },
      camera.projection);
}

// Project the data that is already in the reference frame of the
// target camera.
//
// This method is designed to allow for specialization per target camera
// projection. The interface allows for culling and splitting triangles.
template <typename... T>
auto project(SceneVertexDescriptorList vertices, TriangleDescriptorList triangles,
             std::tuple<std::vector<T>...> attributes, const Metadata::CameraParameters &target) {
  return visit(
      [&](auto const &x) {
        Engine<decay_t<decltype(x)>> engine{target};
        return engine.project(std::move(vertices), std::move(triangles), std::move(attributes));
      },
      target.projection);
}

// Reproject from a source frame with a source camera to a target camera,
// generating lists of vertices, triangles and attributes.
template <typename... T>
auto reproject(const Common::Mat<float> &depth, const Metadata::CameraParameters &camera,
               const Metadata::CameraParameters &target, const Common::Mat<T> &... matrices) {
  auto x = unproject(depth, camera, target, matrices...);
  return project(std::move(std::get<0>(x)), std::move(std::get<1>(x)), std::move(std::get<2>(x)),
                 target);
}

// Unproject a pixel from a source frame to scene coordinates in the reference
// frame of the target camera.
//
// This method is less efficient because of the switch on projection type, but
// suitable for rendering directly from an atlas.
auto unprojectVertex(Common::Vec2f position, float depth, const Metadata::CameraParameters &camera)
    -> Common::Vec3f;
} // namespace TMIV::Renderer

#endif
