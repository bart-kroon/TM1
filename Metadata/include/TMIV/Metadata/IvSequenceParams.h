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

#include "ViewingSpace.h"
#include <TMIV/Common/Json.h>
#include <TMIV/Common/Vector.h>

#include <cassert>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

namespace TMIV::Metadata {
class InputBitstream;
class OutputBitstream;

// In specification: ivs_profile_tier_level( )
struct IvsProfileTierLevel {
  friend std::ostream &operator<<(std::ostream &stream,
                                  const IvsProfileTierLevel &viewParamsVector);
  bool operator==(const IvsProfileTierLevel &other) const;
  bool operator!=(const IvsProfileTierLevel &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> IvsProfileTierLevel;
  void encodeTo(OutputBitstream &) const;
};

struct ErpParams {
  // In specification: erp_phi_min[ v ]
  // In specification: erp_phi_max[ v ]
  Common::Vec2f phiRange{};

  // In specification: erp_theta_min[ v ]
  // In specification: erp_theta_max[ v ]
  Common::Vec2f thetaRange{};

  friend std::ostream &operator<<(std::ostream &stream, const ErpParams &);
  bool operator==(const ErpParams &) const;
  bool operator!=(const ErpParams &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> ErpParams;
  void encodeTo(OutputBitstream &) const;
};

struct PerspectiveParams {
  // In specification: perspective_focal_hor[ v ]
  // In specification: perspective_focal_ver[ v ]
  Common::Vec2f focal{};

  // In specification: perspective_center_hor[ v ]
  // In specification: perspective_center_ver[ v ]
  Common::Vec2f center{};

  friend std::ostream &operator<<(std::ostream &stream, const PerspectiveParams &);
  bool operator==(const PerspectiveParams &) const;
  bool operator!=(const PerspectiveParams &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &) -> PerspectiveParams;
  void encodeTo(OutputBitstream &) const;
};

using ProjectionParams = std::variant<ErpParams, PerspectiveParams>;

struct ViewParams {
  // In specification: projection_plane_width_minus1[ v ]
  // In specification: projection_plane_height_minus1[ v ]
  Common::Vec2i size{};

  // In specification: cam_pos_x[ v ]
  // In specification: cam_pos_y[ v ]
  // In specification: cam_pos_z[ v ]
  Common::Vec3f position{};

  // In specification: cam_yaw[ v ]
  // In specification: cam_pitch[ v ]
  // In specification: cam_roll[ v ]
  Common::Vec3f rotation{};

  // In specification: cam_type[ v ]
  ProjectionParams projection{};
  auto erp() const -> const ErpParams &;
  auto perspective() const -> const PerspectiveParams &;

  // In specification: norm_disp_low[ v ]
  // In specification: norm_disp_high[ v ]
  Common::Vec2f normDispRange{};

  // In specification: depth_occ_map_threshold_default[ v ]
  //
  // Do not set within the encoder! Use encoder.hasInvalidDepth or encoder.wantOccupancy
  // instead. The DepthOccupancy component determines this threshold.
  uint16_t depthOccMapThreshold{};

  // In specification: depth_start_default_present_flag[ v ]
  // In specification: depth_start_default[ v ]
  std::optional<uint16_t> depthStart{};

  // Not part of the bitstream. Improve screen output.
  std::string name{};

  // Not part of the bitstream. Does the source material have invalid depth? The OccupancyTransform
  // class evaluate this field to expand 16-bit depth values.
  bool hasInvalidDepth{};

  // Not part of the bitstream. When an atlas constructor wants to use occupancy for a certain view,
  // then it has to set this data member to true.
  bool wantOccupancy{};

  // The DepthOccupancy component evaluates this member function to decide how to transform the
  // depth values prior to video coding.
  constexpr auto useOccupancy() const -> bool { return hasInvalidDepth || wantOccupancy; }

  // Avoid marking valid depth as invalid depth when enabling occupancy
  constexpr auto avoidInvalidDepth(std::uint16_t x) const {
    return x > 0 || hasInvalidDepth || !useOccupancy() ? x : std::uint16_t(1);
  }

  friend std::ostream &operator<<(std::ostream &stream, const ViewParams &viewParams);
  bool operator==(const ViewParams &other) const;
  bool operator!=(const ViewParams &other) const { return !operator==(other); }

  // Load a single (source) camera from a JSON metadata file (RVS 3.x format)
  //
  // The parameter is a an item of the viewParamsVector node (a JSON object).
  static ViewParams loadFromJson(const Common::Json &node);
};

using ViewParamsVector = std::vector<ViewParams>;

// Data type that corresponds to camera_params_list of specification
struct ViewParamsList : public ViewParamsVector {
  ViewParamsList() = default;
  explicit ViewParamsList(ViewParamsVector cameraParameters)
      : ViewParamsVector{std::move(cameraParameters)} {}
  ViewParamsList(const ViewParamsList &) = default;
  ViewParamsList(ViewParamsList &&) = default;
  ViewParamsList &operator=(const ViewParamsList &) = default;
  ViewParamsList &operator=(ViewParamsList &&) = default;

  // In specification: intrinsic_params_equal_flag
  bool areIntrinsicParamsEqual() const;

  // In specification: depth_quantization_params_equal_flag
  bool areDepthQuantizationParamsEqual() const;

  // Size of each view as a vector
  auto viewSizes() const -> Common::SizeVector;

  friend std::ostream &operator<<(std::ostream &stream, const ViewParamsList &viewParamsVector);
  bool operator==(const ViewParamsList &other) const;
  bool operator!=(const ViewParamsList &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &, unsigned depthOccMapThresholdNumBits) -> ViewParamsList;
  void encodeTo(OutputBitstream &, unsigned depthOccMapThresholdNumBits) const;

  // Load (source) camera parameters from a JSON metadata file (RVS 3.x format)
  // with viewParamsVector specified by name, in that order
  //
  // The first parameter is the viewParamsVector node (a JSON array).
  static ViewParamsList loadFromJson(const Common::Json &node,
                                     const std::vector<std::string> &names);
};

struct IvSequenceParams {
  // In specification: ivs_profile_tier_level( )
  IvsProfileTierLevel ivsProfileTierLevel;

  // In specification: view_params_list( )
  ViewParamsList viewParamsList;

  // In specification: depth_low_quality_flag
  bool depthLowQualityFlag{};

  // In specification: num_groups_minus1
  unsigned numGroups{1};

  // In specification: max_entities_minus1
  unsigned maxEntities{1};

  // In specification: depth_occ_map_threshold_num_bits_minus8
  unsigned depthOccMapThresholdNumBits{10};

  // In specification: viewing_space_present_flag
  // In specification: viewing_space( )
  std::optional<ViewingSpace> viewingSpace{};

  friend std::ostream &operator<<(std::ostream &stream, const IvSequenceParams &ivSequenceParams);
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
