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

#include <TMIV/Common/Json.h>
#include <TMIV/Common/Vector.h>

#include <cassert>
#include <cstdint>
#include <iosfwd>
#include <utility>
#include <variant>
#include <vector>

namespace TMIV::Metadata {
class InputBitstream;
class OutputBitstream;

struct IvsProfileTierLevel {
  // Not yet defined in the specification

  bool operator==(const IvsProfileTierLevel &other) const;
  bool operator!=(const IvsProfileTierLevel &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> IvsProfileTierLevel;
  void encodeTo(OutputBitstream &) const;
};

struct ErpParams {
  // In specification: erp_phi_{min,max}
  Common::Vec2f phiRange{};

  // In specification: erp_theta_{min,max}
  Common::Vec2f thetaRange{};

  friend std::ostream &operator<<(std::ostream &stream, const ErpParams &);
  bool operator==(const ErpParams &) const;
  bool operator!=(const ErpParams &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> ErpParams;
  void encodeTo(OutputBitstream &) const;
};

struct PerspectiveParams {
  // In specification: perspective_focal_{hor,ver}
  Common::Vec2f focal{};

  // In specification: perspective_center_{hor,ver}
  Common::Vec2f center{};

  friend std::ostream &operator<<(std::ostream &stream, const PerspectiveParams &);
  bool operator==(const PerspectiveParams &) const;
  bool operator!=(const PerspectiveParams &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> PerspectiveParams;
  void encodeTo(OutputBitstream &) const;
};

using ProjectionParams = std::variant<ErpParams, PerspectiveParams>;

// Data type that corresponds to an entry of camera_params_list of specification
struct ViewParams {
  // In specification: projection_plane_{width,height}
  Common::Vec2i size{};

  // In specification: cam_pos_{x,y,z}
  Common::Vec3f position{};

  // In specification: cam_{yaw,pitch,roll}
  Common::Vec3f rotation{};

  // In specification: cam_type
  ProjectionParams projection{};
  auto erp() const -> const ErpParams &;
  auto perspective() const -> const PerspectiveParams &;

  // In specification: depth_{near,far}
  Common::Vec2f normDispRange{};

  uint16_t depthOccMapThreshold{};

  friend std::ostream &operator<<(std::ostream &stream, const ViewParams &camera);
  bool operator==(const ViewParams &other) const;
  bool operator!=(const ViewParams &other) const { return !operator==(other); }

  // Load a single (source) camera from a JSON metadata file (RVS 3.x format)
  //
  // The parameter is a an item of the cameras node (a JSON object).
  static ViewParams loadFromJson(const Common::Json &node);
};

using ViewParamsVector = std::vector<ViewParams>;

// No change when depthOccMapThreshold == 0 (no invalid depth)
// Otherwise set depthOccMapThreshold -> 64 and adjust normDispRange
auto modifyDepthRange(const ViewParamsVector &) -> ViewParamsVector;

// Data type that corresponds to camera_params_list of specification
struct CameraParamsList : public ViewParamsVector {
  CameraParamsList() = default;
  explicit CameraParamsList(ViewParamsVector cameraParameters)
      : ViewParamsVector{std::move(cameraParameters)} {}
  CameraParamsList(const CameraParamsList &) = default;
  CameraParamsList(CameraParamsList &&) = default;
  CameraParamsList &operator=(const CameraParamsList &) = default;
  CameraParamsList &operator=(CameraParamsList &&) = default;

  bool areIntrinsicParamsEqual() const;
  bool areDepthQuantizationParamsEqual() const;

  friend std::ostream &operator<<(std::ostream &stream, const CameraParamsList &cameras);
  bool operator==(const CameraParamsList &other) const;
  bool operator!=(const CameraParamsList &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> CameraParamsList;
  void encodeTo(OutputBitstream &) const;

  // Load (source) camera parameters from a JSON metadata file (RVS 3.x format)
  // with cameras specified by name, in that order
  //
  // The first parameter is the cameras node (a JSON array).
  static CameraParamsList loadFromJson(const Common::Json &node,
                                       const std::vector<std::string> &names);
};

struct IvSequenceParams {
  IvsProfileTierLevel ivsProfileTierLevel;
  CameraParamsList cameraParamsList;

  bool operator==(const IvSequenceParams &other) const;
  bool operator!=(const IvSequenceParams &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> IvSequenceParams;
  void encodeTo(OutputBitstream &) const;
};

inline auto ViewParams::erp() const -> const ErpParams & {
  assert(std::holds_alternative<ErpParams>(projection));
  return *std::get_if<ErpParams>(&projection);
}

inline auto ViewParams::perspective() const -> const PerspectiveParams & {
  assert(std::holds_alternative<PerspectiveParams>(projection));
  return *std::get_if<PerspectiveParams>(&projection);
}
} // namespace TMIV::Metadata

#endif
