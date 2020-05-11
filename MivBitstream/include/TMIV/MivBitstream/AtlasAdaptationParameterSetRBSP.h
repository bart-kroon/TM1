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

#ifndef _TMIV_MIVBITSTREAM_ATLASADAPTATIONPARAMETERSETRBSP_H_
#define _TMIV_MIVBITSTREAM_ATLASADAPTATIONPARAMETERSETRBSP_H_

#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Matrix.h>
#include <TMIV/Common/Quaternion.h>
#include <TMIV/Common/Vector.h>

#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::MivBitstream {
// 23090-12: camera_extrinsics()
class CameraExtrinsics {
public:
  [[nodiscard]] constexpr auto ce_view_pos_x() const noexcept;
  [[nodiscard]] constexpr auto ce_view_pos_y() const noexcept;
  [[nodiscard]] constexpr auto ce_view_pos_z() const noexcept;
  [[nodiscard]] constexpr auto ce_view_quat_x() const noexcept;
  [[nodiscard]] constexpr auto ce_view_quat_y() const noexcept;
  [[nodiscard]] constexpr auto ce_view_quat_z() const noexcept;

  constexpr auto ce_view_pos_x(const float value) noexcept -> auto &;
  constexpr auto ce_view_pos_y(const float value) noexcept -> auto &;
  constexpr auto ce_view_pos_z(const float value) noexcept -> auto &;
  constexpr auto ce_view_quat_x(const float value) noexcept -> auto &;
  constexpr auto ce_view_quat_y(const float value) noexcept -> auto &;
  constexpr auto ce_view_quat_z(const float value) noexcept -> auto &;

  [[nodiscard]] auto position() const noexcept -> Common::Vec3f;
  [[nodiscard]] auto rotation() const noexcept -> Common::QuatF;

  auto position(Common::Vec3f) noexcept -> CameraExtrinsics &;
  auto rotation(Common::QuatF) noexcept -> CameraExtrinsics &;

  auto printTo(std::ostream &stream, std::uint16_t viewId) const -> std::ostream &;

  constexpr auto operator==(const CameraExtrinsics &) const noexcept;
  constexpr auto operator!=(const CameraExtrinsics &) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> CameraExtrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  float m_ce_view_pos_x{};
  float m_ce_view_pos_y{};
  float m_ce_view_pos_z{};
  float m_ce_view_quat_x{};
  float m_ce_view_quat_y{};
  float m_ce_view_quat_z{};
};

// 23090-12: ci_cam_type
enum class CiCamType : std::uint8_t { equirectangular, perspective, orthographic };
auto operator<<(std::ostream &, const CiCamType) -> std::ostream &;

using Equirectangular = std::integral_constant<CiCamType, CiCamType::equirectangular>;
using Perspective = std::integral_constant<CiCamType, CiCamType::perspective>;
using Orthographic = std::integral_constant<CiCamType, CiCamType::orthographic>;

// 23090-12: camera_intrinsics()
class CameraIntrinsics {
public:
  [[nodiscard]] constexpr auto ci_cam_type() const noexcept;
  [[nodiscard]] constexpr auto ci_projection_plane_width_minus1() const noexcept;
  [[nodiscard]] constexpr auto ci_projection_plane_height_minus1() const noexcept;

  [[nodiscard]] auto ci_erp_phi_min() const noexcept -> float;
  [[nodiscard]] auto ci_erp_phi_max() const noexcept -> float;
  [[nodiscard]] auto ci_erp_theta_min() const noexcept -> float;
  [[nodiscard]] auto ci_erp_theta_max() const noexcept -> float;
  [[nodiscard]] auto ci_perspective_focal_hor() const noexcept -> float;
  [[nodiscard]] auto ci_perspective_focal_ver() const noexcept -> float;
  [[nodiscard]] auto ci_perspective_center_hor() const noexcept -> float;
  [[nodiscard]] auto ci_perspective_center_ver() const noexcept -> float;
  [[nodiscard]] auto ci_ortho_width() const noexcept -> float;
  [[nodiscard]] auto ci_ortho_height() const noexcept -> float;

  constexpr auto ci_cam_type(const CiCamType value) noexcept -> auto &;
  constexpr auto ci_projection_plane_width_minus1(const std::uint16_t value) noexcept -> auto &;
  constexpr auto ci_projection_plane_height_minus1(const std::uint16_t value) noexcept -> auto &;

  constexpr auto ci_erp_phi_min(const float value) noexcept -> auto &;
  constexpr auto ci_erp_phi_max(const float value) noexcept -> auto &;
  constexpr auto ci_erp_theta_min(const float value) noexcept -> auto &;
  constexpr auto ci_erp_theta_max(const float value) noexcept -> auto &;
  constexpr auto ci_perspective_focal_hor(const float value) noexcept -> auto &;
  constexpr auto ci_perspective_focal_ver(const float value) noexcept -> auto &;
  constexpr auto ci_perspective_center_hor(const float value) noexcept -> auto &;
  constexpr auto ci_perspective_center_ver(const float value) noexcept -> auto &;
  constexpr auto ci_ortho_width(const float value) noexcept -> auto &;
  constexpr auto ci_ortho_height(const float value) noexcept -> auto &;

  [[nodiscard]] auto projectionPlaneSize() const -> Common::Vec2i;

  // Wrap the ci_cam_type in an integral constant and pass a value of that type to the unary
  // function f. This allows to template on the camera projection type.
  template <typename F> decltype(auto) dispatch(F f) const;

  auto printTo(std::ostream &stream, std::uint16_t viewId) const -> std::ostream &;

  constexpr auto operator==(const CameraIntrinsics &) const noexcept;
  constexpr auto operator!=(const CameraIntrinsics &) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> CameraIntrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  CiCamType m_ci_cam_type{};
  std::uint16_t m_ci_projection_plane_width_minus1{};
  std::uint16_t m_ci_projection_plane_height_minus1{};
  std::optional<float> m_ci_erp_phi_min;
  std::optional<float> m_ci_erp_phi_max;
  std::optional<float> m_ci_erp_theta_min;
  std::optional<float> m_ci_erp_theta_max;
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
  [[nodiscard]] constexpr auto dq_depth_occ_map_threshold_default() const noexcept;

  constexpr auto dq_norm_disp_low(const float value) noexcept -> auto &;
  constexpr auto dq_norm_disp_high(const float value) noexcept -> auto &;
  constexpr auto dq_depth_occ_map_threshold_default(const std::uint32_t value) noexcept -> auto &;

  auto printTo(std::ostream &stream, std::uint16_t viewId) const -> std::ostream &;

  constexpr auto operator==(const DepthQuantization &) const noexcept;
  constexpr auto operator!=(const DepthQuantization &) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> DepthQuantization;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  float m_dq_norm_disp_low{};
  float m_dq_norm_disp_high{};
  std::uint32_t m_dq_depth_occ_map_threshold_default{};
};

// 23090-12: pruning_parent()
class PruningParent {
public:
  PruningParent() = default;
  explicit PruningParent(std::vector<std::uint16_t> pp_parent_id);

  [[nodiscard]] auto pp_is_root_flag() const noexcept -> bool;
  [[nodiscard]] auto pp_num_parent_minus1() const noexcept -> std::uint16_t;
  [[nodiscard]] auto pp_parent_id(std::uint16_t i) const noexcept -> std::uint16_t;

  auto pp_parent_id(std::uint16_t i, std::uint16_t value) noexcept -> PruningParent &;

  [[nodiscard]] auto begin() const noexcept { return m_pp_parent_id.begin(); }
  [[nodiscard]] auto end() const noexcept { return m_pp_parent_id.end(); }

  auto printTo(std::ostream &stream, std::uint16_t viewId) const -> std::ostream &;

  auto operator==(const PruningParent &) const noexcept -> bool;
  auto operator!=(const PruningParent &) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream, std::uint16_t mvp_num_views_minus1)
      -> PruningParent;

  void encodeTo(Common::OutputBitstream &bitstream, std::uint16_t mvp_num_views_minus1) const;

private:
  std::vector<std::uint16_t> m_pp_parent_id;
};

// 23090-12: miv_view_params_list()
class MivViewParamsList {
public:
  [[nodiscard]] auto mvp_num_views_minus1() const noexcept -> std::uint16_t;
  [[nodiscard]] constexpr auto mvp_intrinsic_params_equal_flag() const noexcept;
  [[nodiscard]] constexpr auto mvp_depth_quantization_params_equal_flag() const noexcept;
  [[nodiscard]] constexpr auto mvp_pruning_graph_params_present_flag() const noexcept;

  // Return camera extrinsics for the specified view ID.
  [[nodiscard]] auto camera_extrinsics(const std::uint16_t viewId) const noexcept
      -> const CameraExtrinsics &;

  // Return camera intrinsics for the specified view ID. The
  // mvp_intrinsic_params_equal_flag() case is handled for convenience.
  [[nodiscard]] auto camera_intrinsics(std::uint16_t viewId = 0) const noexcept
      -> const CameraIntrinsics &;

  // Return depth quantization for the specified view ID. The
  // mvp_depth_quantization_params_equal_flag() case is handled for convenience.
  [[nodiscard]] auto depth_quantization(std::uint16_t viewId = 0) const noexcept
      -> const DepthQuantization &;

  [[nodiscard]] auto pruning_parent(const std::uint16_t viewId) const noexcept
      -> const PruningParent &;

  // Calling this function will allocate the camera extrinsics list
  auto mvp_num_views_minus1(const std::uint16_t value) noexcept -> MivViewParamsList &;

  // Calling this function will allocate the camera intrinsics list
  auto mvp_intrinsic_params_equal_flag(const bool value) noexcept -> MivViewParamsList &;

  // Calling this function will allocate the depth quantization list
  auto mvp_depth_quantization_params_equal_flag(const bool value) noexcept -> MivViewParamsList &;

  // Calling this function will allocate the pruning graph list
  auto mvp_pruning_graph_params_present_flag(const bool value) noexcept -> MivViewParamsList &;

  [[nodiscard]] auto camera_extrinsics(const std::uint16_t viewId) noexcept -> CameraExtrinsics &;
  [[nodiscard]] auto camera_intrinsics(const std::uint16_t viewId = 0) noexcept
      -> CameraIntrinsics &;
  [[nodiscard]] auto depth_quantization(const std::uint16_t viewId = 0) noexcept
      -> DepthQuantization &;
  [[nodiscard]] auto pruning_parent(const std::uint16_t viewId) noexcept -> PruningParent &;

  friend auto operator<<(std::ostream &stream, const MivViewParamsList &x) -> std::ostream &;

  auto operator==(const MivViewParamsList &) const noexcept -> bool;
  auto operator!=(const MivViewParamsList &) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MivViewParamsList;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  std::vector<CameraExtrinsics> m_camera_extrinsics;
  bool m_mvp_intrinsic_params_equal_flag{};
  std::vector<CameraIntrinsics> m_camera_intrinsics;
  bool m_mvp_depth_quantization_params_equal_flag{};
  std::vector<DepthQuantization> m_depth_quantization;
  bool m_mvp_pruning_graph_params_present_flag{};
  std::vector<PruningParent> m_pruning_parent;
};

// 23090-12: miv_view_params_update_extrinsics
class MivViewParamsUpdateExtrinsics {
public:
  [[nodiscard]] auto mvpue_num_view_updates_minus1() const noexcept -> std::uint16_t;
  [[nodiscard]] auto mvpue_view_idx(const std::uint16_t i) const noexcept -> std::uint16_t;
  [[nodiscard]] auto camera_extrinsics(const std::uint16_t i) const noexcept
      -> const CameraExtrinsics &;
  [[nodiscard]] auto camera_extrinsics(const std::uint16_t i) noexcept -> CameraExtrinsics &;

  // Calling this function will allocate the camera extrinsic update list
  auto mvpue_num_view_updates_minus1(const std::uint16_t value) noexcept
      -> MivViewParamsUpdateExtrinsics &;
  auto mvpue_view_idx(const std::uint16_t i, const std::uint16_t value) noexcept
      -> MivViewParamsUpdateExtrinsics &;

  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateExtrinsics &x)
      -> std::ostream &;

  auto operator==(const MivViewParamsUpdateExtrinsics &) const noexcept -> bool;
  auto operator!=(const MivViewParamsUpdateExtrinsics &) const noexcept -> bool;

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
  [[nodiscard]] auto mvpue_num_view_updates_minus1() const noexcept -> std::uint16_t;
  [[nodiscard]] auto mvpue_view_idx(const std::uint16_t i) const noexcept -> std::uint16_t;
  [[nodiscard]] auto camera_intrinsics(const std::uint16_t i) const noexcept
      -> const CameraIntrinsics &;
  [[nodiscard]] auto camera_intrinsics(const std::uint16_t i) noexcept -> CameraIntrinsics &;

  // Calling this function will allocate the camera intrinsics update list
  auto mvpue_num_view_updates_minus1(const std::uint16_t value) noexcept
      -> MivViewParamsUpdateIntrinsics &;
  auto mvpue_view_idx(const uint16_t i, const std::uint16_t value) noexcept
      -> MivViewParamsUpdateIntrinsics &;

  friend auto operator<<(std::ostream &stream, const MivViewParamsUpdateIntrinsics &x)
      -> std::ostream &;

  auto operator==(const MivViewParamsUpdateIntrinsics &) const noexcept -> bool;
  auto operator!=(const MivViewParamsUpdateIntrinsics &) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> MivViewParamsUpdateIntrinsics;

  void encodeTo(Common::OutputBitstream &bitstream) const;

private:
  uint16_t m_mvpue_num_view_updates_minus1{};
  std::vector<uint16_t> m_mvpue_view_idx;
  std::vector<CameraIntrinsics> m_camera_intrinsics;
};

// 23090-12: ap_miv_view_params_list_update_mode
enum class MvpUpdateMode : std::uint8_t { VPL_INITLIST, VPL_UPD_EXT, VPL_UPD_INT, VPL_EXT_INT };
auto operator<<(std::ostream &stream, const MvpUpdateMode x) -> std::ostream &;

// 23090-5: aaps_vpcc_extension( )
//
// Limitations of this implementation:
//   * aaps_vpcc_camera_parameters_present_flag == 0
class AapsVpccExtension {
public:
  friend auto operator<<(std::ostream &stream, const AapsVpccExtension &x) -> std::ostream &;

  constexpr auto operator==(const AapsVpccExtension &other) const noexcept;
  constexpr auto operator!=(const AapsVpccExtension &other) const noexcept;

  static auto decodeFrom(Common::InputBitstream &bitstream) -> AapsVpccExtension;

  void encodeTo(Common::OutputBitstream &bitstream) const;
};

// 23090-12: aaps_miv_extension( )
class AapsMivExtension {
public:
  [[nodiscard]] constexpr auto aame_omaf_v1_compatible_flag() const noexcept;
  [[nodiscard]] constexpr auto aame_miv_view_params_list_update_mode() const noexcept;
  [[nodiscard]] auto miv_view_params_list() const noexcept -> const MivViewParamsList &;
  [[nodiscard]] auto miv_view_params_update_extrinsics() const noexcept
      -> const MivViewParamsUpdateExtrinsics &;
  [[nodiscard]] auto miv_view_params_update_intrinsics() const noexcept
      -> const MivViewParamsUpdateIntrinsics &;

  constexpr auto aame_omaf_v1_compatible_flag(bool value) noexcept -> auto &;
  constexpr auto aame_miv_view_params_list_update_mode(MvpUpdateMode value) noexcept -> auto &;
  [[nodiscard]] auto miv_view_params_list() noexcept -> MivViewParamsList &;
  [[nodiscard]] auto miv_view_params_update_extrinsics() noexcept
      -> MivViewParamsUpdateExtrinsics &;
  [[nodiscard]] auto miv_view_params_update_intrinsics() noexcept
      -> MivViewParamsUpdateIntrinsics &;

  friend auto operator<<(std::ostream &stream, const AapsMivExtension &x) -> std::ostream &;

  auto operator==(const AapsMivExtension &) const noexcept -> bool;
  auto operator!=(const AapsMivExtension &) const noexcept -> bool;

  static auto decodeFrom(Common::InputBitstream &stream) -> AapsMivExtension;

  void encodeTo(Common::OutputBitstream &stream) const;

private:
  bool m_aame_omaf_v1_compatible_flag{};
  MvpUpdateMode m_aame_miv_view_params_list_update_mode{};
  std::optional<MivViewParamsList> m_miv_view_params_list;
  std::optional<MivViewParamsUpdateExtrinsics> m_miv_view_params_update_extrinsics;
  std::optional<MivViewParamsUpdateIntrinsics> m_miv_view_params_update_intrinsics;
};

// 23090-5: atlas_adapation_parameter_set_rbsp( )
class AtlasAdaptationParameterSetRBSP {
public:
  [[nodiscard]] constexpr auto aaps_atlas_adaptation_parameter_set_id() const noexcept;
  [[nodiscard]] constexpr auto aaps_log2_max_afoc_present_flag() const noexcept;
  [[nodiscard]] auto aaps_log2_max_atlas_frame_order_cnt_lsb_minus4() const noexcept
      -> std::uint8_t;
  [[nodiscard]] constexpr auto aaps_extension_present_flag() const noexcept;
  [[nodiscard]] constexpr auto aaps_vpcc_extension_flag() const noexcept;
  [[nodiscard]] constexpr auto aaps_miv_extension_flag() const noexcept;
  [[nodiscard]] constexpr auto aaps_extension_6bits() const noexcept;
  [[nodiscard]] auto aaps_vpcc_extension() const noexcept -> const AapsVpccExtension &;
  [[nodiscard]] auto aaps_miv_extension() const noexcept -> const AapsMivExtension &;
  [[nodiscard]] auto aapsExtensionData() const noexcept -> const std::vector<bool> &;

  constexpr auto aaps_atlas_adaptation_parameter_set_id(std::uint8_t value) noexcept -> auto &;
  constexpr auto aaps_log2_max_afoc_present_flag(bool value) noexcept -> auto &;
  auto aaps_log2_max_atlas_frame_order_cnt_lsb_minus4(std::uint8_t value) noexcept
      -> AtlasAdaptationParameterSetRBSP &;
  constexpr auto aaps_extension_present_flag(bool value) noexcept -> auto &;
  auto aaps_vpcc_extension_flag(bool value) noexcept -> AtlasAdaptationParameterSetRBSP &;
  auto aaps_miv_extension_flag(bool value) noexcept -> AtlasAdaptationParameterSetRBSP &;
  auto aaps_extension_6bits(std::uint8_t value) noexcept -> AtlasAdaptationParameterSetRBSP &;
  auto aaps_vpcc_extension(const AapsVpccExtension &value) noexcept
      -> AtlasAdaptationParameterSetRBSP &;
  auto aaps_miv_extension(AapsMivExtension value) noexcept -> AtlasAdaptationParameterSetRBSP &;
  auto aapsExtensionData(std::vector<bool> value) noexcept -> AtlasAdaptationParameterSetRBSP &;

  [[nodiscard]] auto aaps_miv_extension() noexcept -> AapsMivExtension &;

  friend auto operator<<(std::ostream &stream, const AtlasAdaptationParameterSetRBSP &x)
      -> std::ostream &;

  auto operator==(const AtlasAdaptationParameterSetRBSP &) const noexcept -> bool;
  auto operator!=(const AtlasAdaptationParameterSetRBSP &) const noexcept -> bool;

  static auto decodeFrom(std::istream &stream) -> AtlasAdaptationParameterSetRBSP;

  void encodeTo(std::ostream &stream) const;

private:
  std::uint8_t m_aaps_atlas_adaptation_parameter_set_id{};
  bool m_aaps_log2_max_afoc_present_flag{};
  std::optional<std::uint8_t> m_aaps_log2_max_atlas_frame_order_cnt_lsb_minus4{};
  bool m_aaps_extension_present_flag{};
  std::optional<bool> m_aaps_vpcc_extension_flag{};
  std::optional<bool> m_aaps_miv_extension_flag{};
  std::optional<std::uint8_t> m_aaps_extension_6bits{};
  std::optional<AapsVpccExtension> m_aaps_vpcc_extension{};
  std::optional<AapsMivExtension> m_aaps_miv_extension{};
  std::optional<std::vector<bool>> m_aapsExtensionData{};
};
} // namespace TMIV::MivBitstream

#include "AtlasAdaptationParameterSetRBSP.hpp"

#endif
