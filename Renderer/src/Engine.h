/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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
#include <TMIV/Metadata/CameraParameterList.h>

namespace TMIV::Renderer {
struct SceneVertexDescriptor {
  Common::Vec3f position; // m, scene point in target reference frame
  float cosRayAngle;      // cosine of ray angles: cos a = <v, w>/|v||w|
};

using SceneVertexDescriptorList = std::vector<SceneVertexDescriptor>;

// Unproject vertices and change reference frame but do not reproject.
//
// This method is designed to allow for specialization per source camera
// projection. The mesh topology may vary per source projection.
//
// Only the extrinsic parameters of the target camera may be used.
auto makeSceneVertexDescriptorList(const Common::Mat<float> &depth,
                                   const Metadata::CameraParameters &camera,
                                   const Metadata::CameraParameters &target)
    -> SceneVertexDescriptorList;

struct ImageVertexDescriptor {
  Common::Vec2f position; // px, position in image (x right, y down)
  float depth;            // m, depth as defined in the target projection
  float cosRayAngle;      // cosine of ray angles: cos a = <v, w>/|v||w|k
};

using ImageVertexDescriptorList = std::vector<ImageVertexDescriptor>;

// Project the scene vertices that are already in the reference frame of the
// target camera.
//
// This method is designed to allow for specialization per target camera
// projection.
auto project(const SceneVertexDescriptorList &sceneDescriptors,
             const Metadata::CameraParameters &target)
    -> ImageVertexDescriptorList;

// Combine makeSceneVertexDescriptorList and project in one call
auto makeImageVertexDescriptorList(const Common::Mat<float> &depth,
                                   const Metadata::CameraParameters &camera,
                                   const Metadata::CameraParameters &target)
    -> ImageVertexDescriptorList;

struct TriangleDescriptor {
  std::array<int, 3> indices; // indices into vertex lists
  float area;                 // px², area before unprojection
};

using TriangleDescriptorList = std::vector<TriangleDescriptor>;

// Complete triangular mesh by connecting vertices.
//
// This method is designed to allow for specialization per source camera
// projection. The mesh topology may vary per source projection.
//
// The area is used to calculate triangle stretching.
auto makeTriangleDescriptorList(const Metadata::CameraParameters &camera)
    -> TriangleDescriptorList;

// Make a vertex attribute list to augment a vertex descriptor list
//
// The most commonly used attribute is a tristimulus
template <class T>
auto makeVertexAttributeList(const Common::Mat<T> &matrix,
                             const Metadata::CameraParameters &camera)
    -> std::vector<T>;

// Return (R, T) such that x -> Rx + t changes reference frame from the source
// camera to the target camera
auto affineParameters(const Metadata::CameraParameters &camera,
                      const Metadata::CameraParameters &target)
    -> std::pair<Common::Mat3x3f, Common::Vec3f>;

// The Engine is fully specialized on projection type
template <Metadata::ProjectionType type> struct Engine {};
} // namespace TMIV::Renderer

#include "Engine.hpp"

#endif
