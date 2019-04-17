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

#ifndef _TMIV_METADATA_CAMERAPARAMETERLIST_H_
#define _TMIV_METADATA_CAMERAPARAMETERLIST_H_

#include <cstdint>
#include <iosfwd>
#include <vector>

#include <TMIV/Common/Json.h>
#include <TMIV/Common/Vector.h>

namespace TMIV::Metadata {
enum class ProjectionType { ERP, CubeMap, Perspective };
enum class CubicMapType { CubeMap, EAC };
using Common::Vec2f;
using Common::Vec2i;
using Common::Vec3f;

// Camera parameters data type (part of MetadataLib)
// Based on working draft description
//
// Read the RVS 3.x manual for interpretation of angles
struct CameraParameters {
  uint16_t id{};
  Vec2i size{};     // Camera sensor size (width, height) in pixels
  Vec3f position{}; // (x, y, z) in meters, OMAF definition
  Vec3f rotation{}; // Euler angles (yaw, pitch, roll), again OMAF
  ProjectionType type{ProjectionType::ERP};
  Vec2f erpPhiRange{};   // Horizontal range in degrees
  Vec2f erpThetaRange{}; // Vertical rnage in degrees
  CubicMapType cubicMapType{CubicMapType::CubeMap};
  Vec2f perspectiveFocal{};  // Focal length
  Vec2f perspectiveCenter{}; // Principle point
  Vec2f depthRange{};        // [near, far]

  friend std::ostream &operator<<(std::ostream &stream,
                                  const CameraParameters &camera);
};

using CameraParameterList = std::vector<CameraParameters>;

bool intrinsicParamsEqual(const CameraParameterList &);

// Load (source) camera parameters from a JSON metadata file (RVS 3.x format)
// with cameras specified by name, in that order
//
// The first parameter is the cameras node (a JSON array).
CameraParameterList loadCamerasFromJson(const Common::Json &node,
                                        const std::vector<std::string> &names);

// Load a single (source) camera from a JSON metadata file (RVS 3.x format)
//
// The parameter is a an item of the cameras node (a JSON object).
CameraParameters loadCameraFromJson(uint16_t id, const Common::Json &node);
} // namespace TMIV::Metadata

#endif
