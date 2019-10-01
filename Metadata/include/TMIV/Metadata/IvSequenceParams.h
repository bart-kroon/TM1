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

#ifndef _TMIV_METADATA_IVSEQUENCEPARAMS_H_
#define _TMIV_METADATA_IVSEQUENCEPARAMS_H_

#include <cstdint>
#include <iosfwd>
#include <utility>
#include <vector>

#include <TMIV/Common/Json.h>
#include <TMIV/Common/Vector.h>

namespace TMIV::Metadata {
enum class ProjectionType { ERP, Perspective };

class InputBitstream;
class OutputBitstream;

struct IvsProfileTierLevel {
  // Not yet defined in the specification

  bool operator==(const IvsProfileTierLevel &other) const;

  static auto decodeFrom(InputBitstream &) -> IvsProfileTierLevel;
  void encodeTo(OutputBitstream &) const;
};

// Data type that corresponds to an entry of camera_params_list of MPEG/N18576
struct CameraParameters {
  // In MPEG/N18576: projection_plane_{width,height}
  Common::Vec2i size{};

  // In MPEG/N18576: cam_pos_{x,y,z}
  Common::Vec3f position{};

  // In MPEG/N18576: cam_{yaw,pitch,roll}
  Common::Vec3f rotation{};

  // In MPEG/N18576: cam_type
  ProjectionType type{ProjectionType::ERP};

  // In MPEG/N18576: erp_phi_{min,max}
  Common::Vec2f erpPhiRange{};

  // In MPEG/N18576: erp_theta_{min,max}
  Common::Vec2f erpThetaRange{};

  // In MPEG/N18576: perspective_focal_{hor,ver}
  Common::Vec2f perspectiveFocal{};

  // In MPEG/N18576: perspective_center_{hor,ver}
  Common::Vec2f perspectiveCenter{};

  // In MPEG/N18576: depth_{near,far}
  Common::Vec2f normDispRange{};

  uint16_t depthOccMapThreshold{};

  friend std::ostream &operator<<(std::ostream &stream, const CameraParameters &camera);
  bool operator==(const CameraParameters &other) const;

  // Load a single (source) camera from a JSON metadata file (RVS 3.x format)
  //
  // The parameter is a an item of the cameras node (a JSON object).
  static CameraParameters loadFromJson(const Common::Json &node);
};

using CameraParametersList = std::vector<CameraParameters>;

// No change when depthOccMapThreshold == 0 (no invalid depth)
// Otherwise set depthOccMapThreshold -> 64 and adjust normDispRange
auto modifyDepthRange(const CameraParametersList&) -> CameraParametersList;

// Data type that corresponds to camera_params_list of MPEG/N18576
struct CameraParamsList : public CameraParametersList {
  CameraParamsList() = default;
  explicit CameraParamsList(CameraParametersList cameraParameters)
      : CameraParametersList{std::move(cameraParameters)} {}
  CameraParamsList(const CameraParamsList &) = default;
  CameraParamsList(CameraParamsList &&) = default;
  CameraParamsList &operator=(const CameraParamsList &) = default;
  CameraParamsList &operator=(CameraParamsList &&) = default;

  bool areIntrinsicParamsEqual() const;
  bool areDepthQuantizationParamsEqual() const;

  friend std::ostream &operator<<(std::ostream &stream, const CameraParamsList &cameras);
  bool operator==(const CameraParamsList &other) const;

  static auto decodeFrom(InputBitstream &) -> CameraParamsList;
  void encodeTo(OutputBitstream &) const;

  // Load (source) camera parameters from a JSON metadata file (RVS 3.x format)
  // with cameras specified by name, in that order
  //
  // The first parameter is the cameras node (a JSON array).
  static CameraParamsList loadFromJson(const Common::Json &node,
                                       const std::vector<std::string> &names);
};

struct IvsParams {
  IvsProfileTierLevel ivsProfileTierLevel;
  CameraParamsList cameraParamsList;

  bool operator==(const IvsParams &other) const;

  static auto decodeFrom(InputBitstream &) -> IvsParams;
  void encodeTo(OutputBitstream &) const;
};
} // namespace TMIV::Metadata

#endif
