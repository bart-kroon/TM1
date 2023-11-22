/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#ifndef TMIV_MIVBITSTREAM_CAFMIVEXTENSION_H
#define TMIV_MIVBITSTREAM_CAFMIVEXTENSION_H

#include "CommonAtlasSequenceParameterSetRBSP.h"
#include "NalUnit.h"
#include "ViewId.h"

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Common.h>
#include <TMIV/Common/Matrix.h>
#include <TMIV/Common/Quaternion.h>
#include <TMIV/Common/Vector.h>
#include <TMIV/MivBitstream/CaptureDeviceInformation.h>

#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
class MivViewParamsList;

class ChromaScaling {
public:
  [[nodiscard]] constexpr auto cs_u_min() const noexcept;
  [[nodiscard]] constexpr auto cs_u_max() const noexcept;
  [[nodiscard]] constexpr auto cs_v_min() const noexcept;
  [[nodiscard]] constexpr auto cs_v_max() const noexcept;

  constexpr auto cs_u_min(uint16_t value) noexcept -> auto &;
  constexpr auto cs_u_max(uint16_t value) noexcept -> auto &;
  constexpr auto cs_v_min(uint16_t value) noexcept -> auto &;
  constexpr auto cs_v_max(uint16_t value) noexcept -> auto &;

  auto printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream &;

  constexpr auto operator==(const ChromaScaling &other) const noexcept;
  constexpr auto operator!=(const ChromaScaling &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream, const MivViewParamsList &mvpl)
      -> ChromaScaling;

  void encodeTo(Common::OutputBitstream &bitstream, const MivViewParamsList &mvpl) const;

private:
  uint16_t m_cs_u_min{};
  uint16_t m_cs_u_max{};
  uint16_t m_cs_v_min{};
  uint16_t m_cs_v_max{};
};

// 23090-12: camera_extrinsics()
class CameraExtrinsics {
public:
  [[nodiscard]] constexpr auto ce_view_pos_x() const noexcept;
  [[nodiscard]] constexpr auto ce_view_pos_y() const noexcept;
  [[nodiscard]] constexpr auto ce_view_pos_z() const noexcept;
  [[nodiscard]] constexpr auto ce_view_quat_x() const noexcept;
  [[nodiscard]] constexpr auto ce_view_quat_y() const noexcept;
  [[nodiscard]] constexpr auto ce_view_quat_z() const noexcept;

  constexpr auto ce_view_pos_x(float value) noexcept -> auto &;
  constexpr auto ce_view_pos_y(float value) noexcept -> auto &;
  constexpr auto ce_view_pos_z(float value) noexcept -> auto &;
  constexpr auto ce_view_quat_x(int32_t value) noexcept -> auto &;
  constexpr auto ce_view_quat_y(int32_t value) noexcept -> auto &;
  constexpr auto ce_view_quat_z(int32_t value) noexcept -> auto &;

  auto printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream &;

  constexpr auto operator==(const CameraExtrinsics &other) const noexcept;
  constexpr auto operator!=(const CameraExtrinsics &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> CameraExtrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  float m_ce_view_pos_x{};
  float m_ce_view_pos_y{};
  float m_ce_view_pos_z{};
  int32_t m_ce_view_quat_x{};
  int32_t m_ce_view_quat_y{};
  int32_t m_ce_view_quat_z{};
};

// 23090-12: ci_cam_type
enum class CiCamType : uint8_t { equirectangular, perspective, orthographic };
auto operator<<(std::ostream & /*stream*/, CiCamType /*x*/) -> std::ostream &;

using Equirectangular = std::integral_constant<CiCamType, CiCamType::equirectangular>;
using Perspective = std::integral_constant<CiCamType, CiCamType::perspective>;
using Orthographic = std::integral_constant<CiCamType, CiCamType::orthographic>;

// 23090-12: camera_intrinsics()
class CameraIntrinsics {
public:
  [[nodiscard]] constexpr auto ci_cam_type() const noexcept;
  [[nodiscard]] constexpr auto ci_projection_plane_width_minus1() const noexcept;
  [[nodiscard]] constexpr auto ci_projection_plane_height_minus1() const noexcept;

  [[nodiscard]] auto ci_erp_phi_min() const -> float;
  [[nodiscard]] auto ci_erp_phi_max() const -> float;
  [[nodiscard]] auto ci_erp_theta_min() const -> float;
  [[nodiscard]] auto ci_erp_theta_max() const -> float;
  [[nodiscard]] auto ci_perspective_focal_hor() const -> float;
  [[nodiscard]] auto ci_perspective_focal_ver() const -> float;
  [[nodiscard]] auto ci_perspective_center_hor() const -> float;
  [[nodiscard]] auto ci_perspective_center_ver() const -> float;
  [[nodiscard]] auto ci_ortho_width() const -> float;
  [[nodiscard]] auto ci_ortho_height() const -> float;

  constexpr auto ci_cam_type(CiCamType value) noexcept -> auto &;
  constexpr auto ci_projection_plane_width_minus1(int32_t value) noexcept -> auto &;
  constexpr auto ci_projection_plane_height_minus1(int32_t value) noexcept -> auto &;

  constexpr auto ci_erp_phi_min(float value) noexcept -> auto &;
  constexpr auto ci_erp_phi_max(float value) noexcept -> auto &;
  constexpr auto ci_erp_theta_min(float value) noexcept -> auto &;
  constexpr auto ci_erp_theta_max(float value) noexcept -> auto &;
  constexpr auto ci_perspective_focal_hor(float value) noexcept -> auto &;
  constexpr auto ci_perspective_focal_ver(float value) noexcept -> auto &;
  constexpr auto ci_perspective_center_hor(float value) noexcept -> auto &;
  constexpr auto ci_perspective_center_ver(float value) noexcept -> auto &;
  constexpr auto ci_ortho_width(float value) noexcept -> auto &;
  constexpr auto ci_ortho_height(float value) noexcept -> auto &;

  [[nodiscard]] auto projectionPlaneSize() const -> Common::Vec2i;
  [[nodiscard]] auto projectionPlaneSizeF() const -> Common::Vec2f;

  // Wrap the ci_cam_type in an integral constant and pass a value of that type to the unary
  // function f. This allows to template on the camera projection type.
  template <typename F> auto dispatch(F f) const -> decltype(auto);

  auto printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream &;

  constexpr auto operator==(const CameraIntrinsics &other) const noexcept;
  constexpr auto operator!=(const CameraIntrinsics &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> CameraIntrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  CiCamType m_ci_cam_type{};
  int32_t m_ci_projection_plane_width_minus1{};
  int32_t m_ci_projection_plane_height_minus1{};
  std::optional<float> m_ci_erp_phi_min{{}};
  std::optional<float> m_ci_erp_phi_max{{}};
  std::optional<float> m_ci_erp_theta_min{{}};
  std::optional<float> m_ci_erp_theta_max{{}};
  std::optional<float> m_ci_perspective_focal_hor;
  std::optional<float> m_ci_perspective_focal_ver;
  std::optional<float> m_ci_perspective_center_hor;
  std::optional<float> m_ci_perspective_center_ver;
  std::optional<float> m_ci_ortho_width;
  std::optional<float> m_ci_ortho_height;
};

// 23090-12: depth_quantization()
class DepthQuantization {
public:
  [[nodiscard]] constexpr auto dq_quantization_law() const noexcept;
  [[nodiscard]] constexpr auto dq_norm_disp_low() const noexcept;
  [[nodiscard]] constexpr auto dq_norm_disp_high() const noexcept;
  [[nodiscard]] constexpr auto dq_depth_occ_threshold_default() const noexcept;
  [[nodiscard]] constexpr auto dq_linear_near() const noexcept;
  [[nodiscard]] constexpr auto dq_linear_far() const noexcept;

  constexpr auto dq_quantization_law(uint8_t value) noexcept -> auto &;
  constexpr auto dq_norm_disp_low(float value) noexcept -> auto &;
  constexpr auto dq_norm_disp_high(float value) noexcept -> auto &;
  constexpr auto dq_depth_occ_threshold_default(uint32_t value) noexcept -> auto &;
  constexpr auto dq_linear_near(float value) noexcept -> auto &;
  constexpr auto dq_linear_far(float value) noexcept -> auto &;

  [[nodiscard]] constexpr auto dq_pivot_count_minus1() const noexcept;
  [[nodiscard]] auto dq_pivot_norm_disp(int32_t i) const noexcept -> float;

  constexpr auto dq_pivot_count_minus1(uint8_t value) noexcept -> auto &;
  auto dq_pivot_norm_disp(int32_t i, float value) noexcept -> DepthQuantization &;

  auto printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream &;

  constexpr auto operator==(const DepthQuantization &other) const noexcept;
  constexpr auto operator!=(const DepthQuantization &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> DepthQuantization;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint8_t m_dq_quantization_law{};
  float m_dq_norm_disp_low{};
  float m_dq_norm_disp_high{};
  uint32_t m_dq_depth_occ_threshold_default{};
  float m_dq_linear_near{};
  float m_dq_linear_far{};

  uint8_t m_dq_pivot_count_minus1{};
  std::vector<float> m_dq_pivot_norm_disp;
};

// 23090-12: pruning_parents()
class PruningParents {
public:
  PruningParents() = default;
  explicit PruningParents(std::vector<uint16_t> pp_parent_idx);

  [[nodiscard]] auto pp_is_root_flag() const noexcept -> bool;
  [[nodiscard]] auto pp_num_parent_minus1() const -> uint16_t;
  [[nodiscard]] auto pp_parent_idx(uint16_t i) const -> uint16_t;

  auto pp_parent_idx(uint16_t i, uint16_t value) noexcept -> PruningParents &;

  [[nodiscard]] auto begin() const noexcept { return m_pp_parent_id.begin(); }
  [[nodiscard]] auto end() const noexcept { return m_pp_parent_id.end(); }

  auto printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream &;

  auto operator==(const PruningParents &other) const noexcept -> bool;
  auto operator!=(const PruningParents &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, uint16_t mvp_num_views_minus1)
      -> PruningParents;

  void encodeTo(Common::OutputBitstream &bitstream, uint16_t mvp_num_views_minus1) const;

private:
  std::vector<uint16_t> m_pp_parent_id;
};

// 23090-12: sensor_extrinsics(v, s)
class SensorExtrinsics {
public:
  [[nodiscard]] constexpr auto se_sensor_pos_x() const noexcept;
  [[nodiscard]] constexpr auto se_sensor_pos_y() const noexcept;
  [[nodiscard]] constexpr auto se_sensor_pos_z() const noexcept;
  [[nodiscard]] constexpr auto se_sensor_quat_x() const noexcept;
  [[nodiscard]] constexpr auto se_sensor_quat_y() const noexcept;
  [[nodiscard]] constexpr auto se_sensor_quat_z() const noexcept;

  constexpr auto se_sensor_pos_x(float value) noexcept -> auto &;
  constexpr auto se_sensor_pos_y(float value) noexcept -> auto &;
  constexpr auto se_sensor_pos_z(float value) noexcept -> auto &;
  constexpr auto se_sensor_quat_x(int32_t value) noexcept -> auto &;
  constexpr auto se_sensor_quat_y(int32_t value) noexcept -> auto &;
  constexpr auto se_sensor_quat_z(int32_t value) noexcept -> auto &;

  auto printTo(std::ostream &stream, uint16_t v, uint16_t s) const -> std::ostream &;

  constexpr auto operator==(const SensorExtrinsics &other) const noexcept;
  constexpr auto operator!=(const SensorExtrinsics &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> SensorExtrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  float m_se_sensor_pos_x{};
  float m_se_sensor_pos_y{};
  float m_se_sensor_pos_z{};
  int32_t m_se_sensor_quat_x{};
  int32_t m_se_sensor_quat_y{};
  int32_t m_se_sensor_quat_z{};
};

// 23090-12: light_source_extrinsics(v, s)
class LightSourceExtrinsics {
public:
  [[nodiscard]] constexpr auto lse_light_source_pos_x() const noexcept;
  [[nodiscard]] constexpr auto lse_light_source_pos_y() const noexcept;
  [[nodiscard]] constexpr auto lse_light_source_pos_z() const noexcept;
  [[nodiscard]] constexpr auto lse_light_source_quat_x() const noexcept;
  [[nodiscard]] constexpr auto lse_light_source_quat_y() const noexcept;
  [[nodiscard]] constexpr auto lse_light_source_quat_z() const noexcept;

  constexpr auto lse_light_source_pos_x(float value) noexcept -> auto &;
  constexpr auto lse_light_source_pos_y(float value) noexcept -> auto &;
  constexpr auto lse_light_source_pos_z(float value) noexcept -> auto &;
  constexpr auto lse_light_source_quat_x(int32_t value) noexcept -> auto &;
  constexpr auto lse_light_source_quat_y(int32_t value) noexcept -> auto &;
  constexpr auto lse_light_source_quat_z(int32_t value) noexcept -> auto &;

  auto printTo(std::ostream &stream, uint16_t v, uint16_t s) const -> std::ostream &;

  constexpr auto operator==(const LightSourceExtrinsics &other) const noexcept;
  constexpr auto operator!=(const LightSourceExtrinsics &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> LightSourceExtrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  float m_lse_light_source_pos_x{};
  float m_lse_light_source_pos_y{};
  float m_lse_light_source_pos_z{};
  int32_t m_lse_light_source_quat_x{};
  int32_t m_lse_light_source_quat_y{};
  int32_t m_lse_light_source_quat_z{};
};

// 23090-12: DistortionParameters(v, s)
class DistortionParameters {
public:
  [[nodiscard]] constexpr auto dp_model_id() const noexcept;
  [[nodiscard]] auto dp_coefficient(uint8_t i) const -> float;

  auto dp_model_id(uint8_t value) -> DistortionParameters &;
  auto dp_coefficient(uint8_t i, float value) -> DistortionParameters &;

  auto printTo(std::ostream &stream, uint16_t v, uint16_t s) const -> std::ostream &;

  constexpr auto operator==(const DistortionParameters &other) const noexcept;
  constexpr auto operator!=(const DistortionParameters &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> DistortionParameters;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint8_t m_dp_model_id{};
  std::vector<float> m_dp_coefficient{};
};

// 23090-12: miv_view_params_list()
class MivViewParamsList {
public:
  [[nodiscard]] auto mvp_num_views_minus1() const -> uint16_t;

  [[nodiscard]] constexpr auto mvp_explicit_view_id_flag() const noexcept;
  [[nodiscard]] auto mvp_view_id(uint16_t viewIdx) const -> ViewId;
  [[nodiscard]] auto mvp_inpaint_flag(uint16_t viewIdx) const -> bool;
  [[nodiscard]] constexpr auto mvp_intrinsic_params_equal_flag() const noexcept;
  [[nodiscard]] auto mvp_depth_quantization_params_equal_flag() const -> bool;
  [[nodiscard]] constexpr auto mvp_pruning_graph_params_present_flag() const noexcept;
  [[nodiscard]] constexpr auto mvp_depth_reprojection_flag() const noexcept;
  [[nodiscard]] auto mvp_chroma_scaling_bit_depth_minus1() const -> uint8_t;
  [[nodiscard]] auto mvp_view_background_flag(uint16_t viewIdx) const -> bool;

  [[nodiscard]] auto camera_extrinsics(uint16_t viewIdx) const -> const CameraExtrinsics &;

  // Return camera intrinsics for the specified view ID. The
  // mvp_intrinsic_params_equal_flag() case is handled for convenience.
  [[nodiscard]] auto camera_intrinsics(uint16_t viewIdx = 0) const -> const CameraIntrinsics &;

  // Return depth quantization for the specified view ID. The
  // mvp_depth_quantization_params_equal_flag() case is handled for convenience.
  [[nodiscard]] auto depth_quantization(uint16_t viewIdx = 0) const -> const DepthQuantization &;

  [[nodiscard]] auto pruning_parent(uint16_t viewIdx) const -> const PruningParents &;

  [[nodiscard]] auto chroma_scaling(uint16_t viewIdx = 0) const -> const ChromaScaling &;

  [[nodiscard]] auto mvp_device_model_id(uint16_t v) const -> uint8_t;
  [[nodiscard]] auto sensor_extrinsics(uint16_t v, uint16_t s) const -> const SensorExtrinsics &;
  [[nodiscard]] auto distortion_parameters(uint16_t v, uint16_t s) const
      -> const DistortionParameters &;
  [[nodiscard]] auto light_source_extrinsics(uint16_t v, uint16_t s) const
      -> const LightSourceExtrinsics &;

  // Calling this function will allocate the camera extrinsics list
  auto mvp_num_views_minus1(uint16_t value) -> MivViewParamsList &;
  constexpr auto mvp_depth_reprojection_flag(bool value) noexcept -> auto &;

  auto mvp_explicit_view_id_flag(bool value) noexcept -> MivViewParamsList &;
  auto mvp_view_id(uint16_t viewIdx, ViewId viewId) -> MivViewParamsList &;
  auto mvp_inpaint_flag(uint16_t viewIdx, bool value) -> MivViewParamsList &;

  // Calling this function will allocate the camera intrinsics list
  auto mvp_intrinsic_params_equal_flag(bool value) -> MivViewParamsList &;

  // Calling this function will allocate the depth quantization list
  auto mvp_depth_quantization_params_equal_flag(bool value) -> MivViewParamsList &;

  // Calling this function will allocate the pruning graph list
  auto mvp_pruning_graph_params_present_flag(bool value) -> MivViewParamsList &;

  auto mvp_chroma_scaling_bit_depth_minus1(uint8_t value) -> MivViewParamsList &;

  auto mvp_view_background_flag(uint16_t viewIdx, bool value) -> MivViewParamsList &;

  [[nodiscard]] auto camera_extrinsics(uint16_t viewIdx) noexcept -> CameraExtrinsics &;
  [[nodiscard]] auto camera_intrinsics(uint16_t viewIdx = 0) noexcept -> CameraIntrinsics &;
  [[nodiscard]] auto depth_quantization(uint16_t viewIdx = 0) noexcept -> DepthQuantization &;
  [[nodiscard]] auto pruning_parent(uint16_t viewIdx) -> PruningParents &;
  [[nodiscard]] auto chroma_scaling(uint16_t viewIdx) noexcept -> ChromaScaling &;
  auto mvp_device_model_id(uint16_t v, uint8_t value) -> MivViewParamsList &;
  auto sensor_extrinsics(uint16_t v, uint16_t s) -> SensorExtrinsics &;
  auto distortion_parameters(uint16_t v, uint16_t s) -> DistortionParameters &;
  auto light_source_extrinsics(uint16_t v, uint16_t s) -> LightSourceExtrinsics &;

  auto printTo(std::ostream &stream, const CommonAtlasSequenceParameterSetRBSP &casps) const
      -> std::ostream &;

  auto operator==(const MivViewParamsList &other) const noexcept -> bool;
  auto operator!=(const MivViewParamsList &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream,
                         const CommonAtlasSequenceParameterSetRBSP &casps) -> MivViewParamsList;

  void encodeTo(Common::OutputBitstream &bitstream,
                const CommonAtlasSequenceParameterSetRBSP &casps) const;

private:
  bool m_mvp_explicit_view_id_flag{};
  std::vector<ViewId> m_mvp_view_id;
  std::vector<bool> m_mvpInpaintFlag{false};
  std::vector<bool> m_mvpViewBackgroundFlag{false};
  std::vector<CameraExtrinsics> m_camera_extrinsics{{}};
  bool m_mvp_intrinsic_params_equal_flag{};
  std::vector<CameraIntrinsics> m_camera_intrinsics{{}};
  std::optional<bool> m_mvp_depth_quantization_params_equal_flag{};
  std::vector<DepthQuantization> m_depth_quantization{{}};
  bool m_mvp_pruning_graph_params_present_flag{};
  std::vector<PruningParents> m_pruning_parent{};
  std::vector<ChromaScaling> m_mvp_chroma_scaling_values{{}};
  std::optional<bool> m_mvp_depth_reprojection_flag{};
  std::optional<uint8_t> m_mvp_chroma_scaling_bit_depth_minus1{};

  std::vector<uint8_t> m_mvp_device_model_id{0};
  std::vector<std::vector<SensorExtrinsics>> m_sensor_extrinsics{{}};
  std::vector<std::vector<DistortionParameters>> m_distortion_parameters{{}};
  std::vector<std::vector<LightSourceExtrinsics>> m_light_source_extrinsics{{}};
};

// 23090-12: miv_view_params_update_extrinsics
class MivViewParamsUpdateExtrinsics {
public:
  [[nodiscard]] auto mvpue_num_view_updates_minus1() const noexcept -> uint16_t;
  [[nodiscard]] auto mvpue_view_idx(uint16_t i) const -> uint16_t;
  [[nodiscard]] auto camera_extrinsics(uint16_t i) const -> const CameraExtrinsics &;
  [[nodiscard]] auto camera_extrinsics(uint16_t i) noexcept -> CameraExtrinsics &;

  // Calling this function will allocate the camera extrinsic update list
  auto mvpue_num_view_updates_minus1(uint16_t value) -> MivViewParamsUpdateExtrinsics &;
  auto mvpue_view_idx(uint16_t i, uint16_t value) noexcept -> MivViewParamsUpdateExtrinsics &;

  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateExtrinsics &x)
      -> std::ostream &;

  auto operator==(const MivViewParamsUpdateExtrinsics &other) const noexcept -> bool;
  auto operator!=(const MivViewParamsUpdateExtrinsics &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MivViewParamsUpdateExtrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint16_t m_mvpue_num_view_updates_minus1{};
  std::vector<uint16_t> m_mvpue_view_idx;
  std::vector<CameraExtrinsics> m_camera_extrinsics;
};

// 23090-12: miv_view_params_update_intrinsics
class MivViewParamsUpdateIntrinsics {
public:
  [[nodiscard]] auto mvpui_num_view_updates_minus1() const noexcept -> uint16_t;
  [[nodiscard]] auto mvpui_view_idx(uint16_t i) const -> uint16_t;
  [[nodiscard]] auto camera_intrinsics(uint16_t i) const -> const CameraIntrinsics &;
  [[nodiscard]] auto camera_intrinsics(uint16_t i) noexcept -> CameraIntrinsics &;

  // Calling this function will allocate the camera intrinsics update list
  auto mvpui_num_view_updates_minus1(uint16_t value) -> MivViewParamsUpdateIntrinsics &;
  auto mvpui_view_idx(uint16_t i, uint16_t value) noexcept -> MivViewParamsUpdateIntrinsics &;

  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateIntrinsics &x)
      -> std::ostream &;

  auto operator==(const MivViewParamsUpdateIntrinsics &other) const noexcept -> bool;
  auto operator!=(const MivViewParamsUpdateIntrinsics &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MivViewParamsUpdateIntrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint16_t m_mvpui_num_view_updates_minus1{};
  std::vector<uint16_t> m_mvpui_view_idx{std::vector<uint16_t>(1U)};
  std::vector<CameraIntrinsics> m_camera_intrinsics{std::vector<CameraIntrinsics>(1U)};
};

// 23090-12: miv_view_params_update_depth_quantization
class MivViewParamsUpdateDepthQuantization {
public:
  [[nodiscard]] auto mvpudq_num_view_updates_minus1() const noexcept -> uint16_t;
  [[nodiscard]] auto mvpudq_view_idx(uint16_t i) const -> uint16_t;
  [[nodiscard]] auto depth_quantization(uint16_t i) const -> const DepthQuantization &;
  [[nodiscard]] auto depth_quantization(uint16_t i) noexcept -> DepthQuantization &;

  // Calling this function will allocate the depth quantization update list
  auto mvpudq_num_view_updates_minus1(uint16_t value) -> MivViewParamsUpdateDepthQuantization &;
  auto mvpudq_view_idx(uint16_t i, uint16_t value) noexcept
      -> MivViewParamsUpdateDepthQuantization &;

  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateDepthQuantization &x)
      -> std::ostream &;

  auto operator==(const MivViewParamsUpdateDepthQuantization &other) const noexcept -> bool;
  auto operator!=(const MivViewParamsUpdateDepthQuantization &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MivViewParamsUpdateDepthQuantization;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint16_t m_mvpudq_num_view_updates_minus1{};
  std::vector<uint16_t> m_mvpudq_view_idx{};
  std::vector<DepthQuantization> m_depth_quantization;
};

class MivViewParamsUpdateChromaScaling {
public:
  [[nodiscard]] auto mvpucs_num_view_updates_minus1() const noexcept -> uint16_t;
  [[nodiscard]] auto mvpucs_view_idx(uint16_t i) const -> uint16_t;
  [[nodiscard]] auto chroma_scaling(uint16_t i) const -> const ChromaScaling &;
  [[nodiscard]] auto chroma_scaling(uint16_t i) noexcept -> ChromaScaling &;

  // Calling this function will allocate the depth quantization update list
  auto mvpucs_num_view_updates_minus1(uint16_t value) -> MivViewParamsUpdateChromaScaling &;
  auto mvpucs_view_idx(uint16_t i, uint16_t value) noexcept -> MivViewParamsUpdateChromaScaling &;

  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateChromaScaling &x)
      -> std::ostream &;

  auto operator==(const MivViewParamsUpdateChromaScaling &other) const noexcept -> bool;
  auto operator!=(const MivViewParamsUpdateChromaScaling &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const MivViewParamsList &mvpl)
      -> MivViewParamsUpdateChromaScaling;

  void encodeTo(Common::OutputBitstream &bitstream, const MivViewParamsList &mvpl) const;

private:
  uint16_t m_mvpucs_num_view_updates_minus1{};
  std::vector<uint16_t> m_mvpucs_view_idx{};
  std::vector<ChromaScaling> m_chroma_scaling;
};

// 23090-12
class MivViewParamsUpdateSensorExtrinsics {
public:
  [[nodiscard]] auto mvpuse_num_updates_minus1() const noexcept -> uint16_t;
  [[nodiscard]] auto mvpuse_view_idx(uint16_t i) const -> uint16_t;
  [[nodiscard]] auto mvpuse_sensor_idx(uint16_t i) const -> uint16_t;
  [[nodiscard]] auto sensor_extrinsics(uint16_t v, uint16_t s) const -> const SensorExtrinsics &;
  [[nodiscard]] auto sensor_extrinsics(uint16_t v, uint16_t s) noexcept -> SensorExtrinsics &;

  auto mvpuse_num_updates_minus1(uint16_t value) -> MivViewParamsUpdateSensorExtrinsics &;
  auto mvpuse_view_idx(uint16_t i, uint16_t value) noexcept
      -> MivViewParamsUpdateSensorExtrinsics &;
  auto mvpuse_sensor_idx(uint16_t i, uint16_t value) noexcept
      -> MivViewParamsUpdateSensorExtrinsics &;

  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateSensorExtrinsics &x)
      -> std::ostream &;

  auto operator==(const MivViewParamsUpdateSensorExtrinsics &other) const noexcept -> bool;
  auto operator!=(const MivViewParamsUpdateSensorExtrinsics &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MivViewParamsUpdateSensorExtrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint16_t m_mvpuse_num_updates_minus1{};
  std::vector<uint16_t> m_mvpuse_view_idx{};
  std::vector<uint16_t> m_mvpuse_sensor_idx{};
  std::vector<std::vector<SensorExtrinsics>> m_sensor_extrinsics{};
};

// 23090-12
class MivViewParamsUpdateLightSourceExtrinsics {
public:
  [[nodiscard]] auto mvpulse_num_updates_minus1() const noexcept -> uint16_t;
  [[nodiscard]] auto mvpulse_view_idx(uint16_t i) const -> uint16_t;
  [[nodiscard]] auto mvpulse_sensor_idx(uint16_t i) const -> uint16_t;
  [[nodiscard]] auto light_source_extrinsics(uint16_t v, uint16_t s) const
      -> const LightSourceExtrinsics &;
  [[nodiscard]] auto light_source_extrinsics(uint16_t v, uint16_t s) noexcept
      -> LightSourceExtrinsics &;

  auto mvpulse_num_updates_minus1(uint16_t value) -> MivViewParamsUpdateLightSourceExtrinsics &;
  auto mvpulse_view_idx(uint16_t i, uint16_t value) noexcept
      -> MivViewParamsUpdateLightSourceExtrinsics &;
  auto mvpulse_sensor_idx(uint16_t i, uint16_t value) noexcept
      -> MivViewParamsUpdateLightSourceExtrinsics &;

  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateLightSourceExtrinsics &x)
      -> std::ostream &;

  auto operator==(const MivViewParamsUpdateLightSourceExtrinsics &other) const noexcept -> bool;
  auto operator!=(const MivViewParamsUpdateLightSourceExtrinsics &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream)
      -> MivViewParamsUpdateLightSourceExtrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint16_t m_mvpulse_num_updates_minus1{};
  std::vector<uint16_t> m_mvpulse_view_idx{};
  std::vector<uint16_t> m_mvpulse_sensor_idx{};
  std::vector<std::vector<LightSourceExtrinsics>> m_light_source_extrinsics{};
};

// 23090-12
class MivViewParamsUpdateDistortionParameters {
public:
  [[nodiscard]] auto mvpudp_num_updates_minus1() const noexcept -> uint16_t;
  [[nodiscard]] auto mvpudp_view_idx(uint16_t i) const -> uint16_t;
  [[nodiscard]] auto mvpudp_sensor_idx(uint16_t i) const -> uint16_t;
  [[nodiscard]] auto distortion_parameters(uint16_t v, uint16_t s) const
      -> const DistortionParameters &;
  [[nodiscard]] auto distortion_parameters(uint16_t v, uint16_t s) noexcept
      -> DistortionParameters &;

  auto mvpudp_num_updates_minus1(uint16_t value) -> MivViewParamsUpdateDistortionParameters &;
  auto mvpudp_view_idx(uint16_t i, uint16_t value) noexcept
      -> MivViewParamsUpdateDistortionParameters &;
  auto mvpudp_sensor_idx(uint16_t i, uint16_t value) noexcept
      -> MivViewParamsUpdateDistortionParameters &;

  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateDistortionParameters &x)
      -> std::ostream &;

  auto operator==(const MivViewParamsUpdateDistortionParameters &other) const noexcept -> bool;
  auto operator!=(const MivViewParamsUpdateDistortionParameters &other) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream)
      -> MivViewParamsUpdateDistortionParameters;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint16_t m_mvpudp_num_updates_minus1{};
  std::vector<uint16_t> m_mvpudp_view_idx{};
  std::vector<uint16_t> m_mvpudp_sensor_idx{};
  std::vector<std::vector<DistortionParameters>> m_distortion_parameters{};
};

// 23090-12: caf_miv_extension( )
class CafMivExtension {
public:
  [[nodiscard]] auto came_update_extrinsics_flag() const -> bool;
  [[nodiscard]] auto came_update_intrinsics_flag() const -> bool;
  [[nodiscard]] auto came_update_depth_quantization_flag() const -> bool;
  [[nodiscard]] auto came_update_chroma_scaling_flag() const -> bool;
  [[nodiscard]] auto came_update_sensor_extrinsics_flag() const -> bool;
  [[nodiscard]] auto came_update_distortion_parameters_flag() const -> bool;
  [[nodiscard]] auto came_update_light_source_extrinsics_flag() const -> bool;

  [[nodiscard]] auto miv_view_params_list() const -> const MivViewParamsList &;
  [[nodiscard]] auto miv_view_params_update_extrinsics() const
      -> const MivViewParamsUpdateExtrinsics &;
  [[nodiscard]] auto miv_view_params_update_intrinsics() const
      -> const MivViewParamsUpdateIntrinsics &;
  [[nodiscard]] auto miv_view_params_update_depth_quantization() const
      -> const MivViewParamsUpdateDepthQuantization &;
  [[nodiscard]] auto miv_view_params_update_chroma_scaling() const
      -> const MivViewParamsUpdateChromaScaling &;
  [[nodiscard]] auto miv_view_params_update_sensor_extrinsics() const
      -> const MivViewParamsUpdateSensorExtrinsics &;
  [[nodiscard]] auto miv_view_params_update_distortion_parameters() const
      -> const MivViewParamsUpdateDistortionParameters &;
  [[nodiscard]] auto miv_view_params_update_light_source_extrinsics() const
      -> const MivViewParamsUpdateLightSourceExtrinsics &;

  auto came_update_extrinsics_flag(bool value) noexcept -> CafMivExtension &;
  auto came_update_intrinsics_flag(bool value) noexcept -> CafMivExtension &;
  auto came_update_depth_quantization_flag(bool value) noexcept -> CafMivExtension &;
  auto came_update_chroma_scaling_flag(bool value) noexcept -> CafMivExtension &;
  auto came_update_sensor_extrinsics_flag(bool value) noexcept -> CafMivExtension &;
  auto came_update_distortion_parameters_flag(bool value) noexcept -> CafMivExtension &;
  auto came_update_light_source_extrinsics_flag(bool value) noexcept -> CafMivExtension &;

  [[nodiscard]] auto miv_view_params_list() noexcept -> MivViewParamsList &;
  [[nodiscard]] auto miv_view_params_update_extrinsics() noexcept
      -> MivViewParamsUpdateExtrinsics &;
  [[nodiscard]] auto miv_view_params_update_intrinsics() noexcept
      -> MivViewParamsUpdateIntrinsics &;
  [[nodiscard]] auto miv_view_params_update_depth_quantization() noexcept
      -> MivViewParamsUpdateDepthQuantization &;
  [[nodiscard]] auto miv_view_params_update_chroma_scaling() noexcept
      -> MivViewParamsUpdateChromaScaling &;
  [[nodiscard]] auto miv_view_params_update_sensor_extrinsics() noexcept
      -> MivViewParamsUpdateSensorExtrinsics &;
  [[nodiscard]] auto miv_view_params_update_distortion_parameters() noexcept
      -> MivViewParamsUpdateDistortionParameters &;
  [[nodiscard]] auto miv_view_params_update_light_source_extrinsics() noexcept
      -> MivViewParamsUpdateLightSourceExtrinsics &;

  auto printTo(std::ostream &stream, const CommonAtlasSequenceParameterSetRBSP &casps) const
      -> std::ostream &;

  auto operator==(const CafMivExtension &other) const -> bool;
  auto operator!=(const CafMivExtension &other) const -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, const NalUnitHeader &nuh,
                         const CommonAtlasSequenceParameterSetRBSP &casps) -> CafMivExtension;

  void encodeTo(Common::OutputBitstream &bitstream, const NalUnitHeader &nuh,
                const CommonAtlasSequenceParameterSetRBSP &casps) const;

private:
  std::optional<bool> m_came_update_extrinsics_flag{};
  std::optional<bool> m_came_update_intrinsics_flag{};
  std::optional<bool> m_came_update_depth_quantization_flag{};
  std::optional<bool> m_came_update_chroma_scaling_flag{};
  std::optional<bool> m_came_update_sensor_extrinsics_flag{};
  std::optional<bool> m_came_update_distortion_parameters_flag{};
  std::optional<bool> m_came_update_light_source_extrinsics_flag{};
  std::optional<MivViewParamsList> m_miv_view_params_list{};
  std::optional<MivViewParamsUpdateExtrinsics> m_miv_view_params_update_extrinsics;
  std::optional<MivViewParamsUpdateIntrinsics> m_miv_view_params_update_intrinsics;
  std::optional<MivViewParamsUpdateDepthQuantization> m_miv_view_params_update_depth_quantization;
  std::optional<MivViewParamsUpdateChromaScaling> m_miv_view_params_update_chroma_scaling;
  std::optional<MivViewParamsUpdateSensorExtrinsics> m_miv_view_params_update_sensor_extrinsics;
  std::optional<MivViewParamsUpdateDistortionParameters>
      m_miv_view_params_update_distortion_parameters;
  std::optional<MivViewParamsUpdateLightSourceExtrinsics>
      m_miv_view_params_update_light_source_extrinsics;
};

} // namespace TMIV::MivBitstream

#include "CafMivExtension.hpp"

#endif
