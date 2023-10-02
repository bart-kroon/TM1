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

#include <TMIV/MivBitstream/CafMivExtension.h>
#include <TMIV/MivBitstream/CaptureDeviceInformation.h>
#include <TMIV/MivBitstream/CommonAtlasSequenceParameterSetRBSP.h>
#include <TMIV/MivBitstream/Formatters.h>

#include <cmath>

namespace TMIV::MivBitstream {
auto operator<<(std::ostream &stream, const CiCamType x) -> std::ostream & {
  switch (x) {
  case CiCamType::equirectangular:
    return stream << "equirectangular";
  case CiCamType::perspective:
    return stream << "perspective";
  case CiCamType::orthographic:
    return stream << "orthographic";
  default:
    MIVBITSTREAM_ERROR("Unknown cam type");
  }
}

auto CameraIntrinsics::projectionPlaneSize() const -> Common::Vec2i {
  return {ci_projection_plane_width_minus1() + 1, ci_projection_plane_height_minus1() + 1};
}

auto CameraIntrinsics::projectionPlaneSizeF() const -> Common::Vec2f {
  return {static_cast<float>(ci_projection_plane_width_minus1() + 1),
          static_cast<float>(ci_projection_plane_height_minus1() + 1)};
}

auto CameraIntrinsics::ci_erp_phi_min() const -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_phi_min.has_value());
  return *m_ci_erp_phi_min;
}

auto CameraIntrinsics::ci_erp_phi_max() const -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_phi_max.has_value());
  return *m_ci_erp_phi_max;
}

auto CameraIntrinsics::ci_erp_theta_min() const -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_theta_min.has_value());
  return *m_ci_erp_theta_min;
}

auto CameraIntrinsics::ci_erp_theta_max() const -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_theta_max.has_value());
  return *m_ci_erp_theta_max;
}

auto CameraIntrinsics::ci_perspective_focal_hor() const -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_focal_hor.has_value());
  return *m_ci_perspective_focal_hor;
}

auto CameraIntrinsics::ci_perspective_focal_ver() const -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_focal_ver.has_value());
  return *m_ci_perspective_focal_ver;
}

auto CameraIntrinsics::ci_perspective_center_hor() const -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_center_hor.has_value());
  return *m_ci_perspective_center_hor;
}

auto CameraIntrinsics::ci_perspective_center_ver() const -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_center_ver.has_value());
  return *m_ci_perspective_center_ver;
}

auto CameraIntrinsics::ci_ortho_width() const -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::orthographic);
  VERIFY_MIVBITSTREAM(m_ci_ortho_width.has_value());
  return *m_ci_ortho_width;
}

auto CameraIntrinsics::ci_ortho_height() const -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::orthographic);
  VERIFY_MIVBITSTREAM(m_ci_ortho_height.has_value());
  return *m_ci_ortho_height;
}

auto CameraIntrinsics::printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream & {
  stream << "ci_cam_type[ " << viewIdx << " ]=" << ci_cam_type() << '\n';
  stream << "ci_projection_plane_width_minus1[ " << viewIdx
         << " ]=" << ci_projection_plane_width_minus1() << '\n';
  stream << "ci_projection_plane_height_minus1[ " << viewIdx
         << " ]=" << ci_projection_plane_height_minus1() << '\n';

  switch (ci_cam_type()) {
  case CiCamType::equirectangular:
    stream << "ci_erp_phi_min[ " << viewIdx << " ]=" << ci_erp_phi_min() << '\n';
    stream << "ci_erp_phi_max[ " << viewIdx << " ]=" << ci_erp_phi_max() << '\n';
    stream << "ci_erp_theta_min[ " << viewIdx << " ]=" << ci_erp_theta_min() << '\n';
    stream << "ci_erp_theta_max[ " << viewIdx << " ]=" << ci_erp_theta_max() << '\n';
    return stream;

  case CiCamType::perspective:
    stream << "ci_perspective_focal_hor[ " << viewIdx << " ]=" << ci_perspective_focal_hor()
           << '\n';
    stream << "ci_perspective_focal_ver[ " << viewIdx << " ]=" << ci_perspective_focal_ver()
           << '\n';
    stream << "ci_perspective_center_hor[ " << viewIdx << " ]=" << ci_perspective_center_hor()
           << '\n';
    stream << "ci_perspective_center_ver[ " << viewIdx << " ]=" << ci_perspective_center_ver()
           << '\n';
    return stream;

  case CiCamType::orthographic:
    stream << "ci_ortho_width[ " << viewIdx << " ]=" << ci_ortho_width() << '\n';
    stream << "ci_ortho_height[ " << viewIdx << " ]=" << ci_ortho_height() << '\n';
    return stream;

  default:
    MIVBITSTREAM_ERROR("Unknown cam type");
  }
}

auto CameraIntrinsics::decodeFrom(Common::InputBitstream &bitstream) -> CameraIntrinsics {
  auto x = CameraIntrinsics{};

  x.ci_cam_type(CiCamType(bitstream.getUint8()));
  x.ci_projection_plane_width_minus1(bitstream.getUint16());
  x.ci_projection_plane_height_minus1(bitstream.getUint16());

  switch (x.ci_cam_type()) {
  case CiCamType::equirectangular:
    x.ci_erp_phi_min(bitstream.getFloat32());
    x.ci_erp_phi_max(bitstream.getFloat32());
    x.ci_erp_theta_min(bitstream.getFloat32());
    x.ci_erp_theta_max(bitstream.getFloat32());
    return x;

  case CiCamType::perspective:
    x.ci_perspective_focal_hor(bitstream.getFloat32());
    x.ci_perspective_focal_ver(bitstream.getFloat32());
    x.ci_perspective_center_hor(bitstream.getFloat32());
    x.ci_perspective_center_ver(bitstream.getFloat32());
    return x;

  case CiCamType::orthographic:
    x.ci_ortho_width(bitstream.getFloat32());
    x.ci_ortho_height(bitstream.getFloat32());
    return x;

  default:
    MIVBITSTREAM_ERROR("Unknown cam type");
  }
}

void CameraIntrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint8(static_cast<uint8_t>(ci_cam_type()));

  bitstream.putUint16(Common::downCast<uint16_t>(ci_projection_plane_width_minus1()));
  bitstream.putUint16(Common::downCast<uint16_t>(ci_projection_plane_height_minus1()));

  switch (ci_cam_type()) {
  case CiCamType::equirectangular:
    bitstream.putFloat32(ci_erp_phi_min());
    bitstream.putFloat32(ci_erp_phi_max());
    bitstream.putFloat32(ci_erp_theta_min());
    bitstream.putFloat32(ci_erp_theta_max());
    return;

  case CiCamType::perspective:
    bitstream.putFloat32(ci_perspective_focal_hor());
    bitstream.putFloat32(ci_perspective_focal_ver());
    bitstream.putFloat32(ci_perspective_center_hor());
    bitstream.putFloat32(ci_perspective_center_ver());
    return;

  case CiCamType::orthographic:
    bitstream.putFloat32(ci_ortho_width());
    bitstream.putFloat32(ci_ortho_height());
    return;

  default:
    UNREACHABLE;
  }
}

auto ChromaScaling::printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream & {
  fmt::print(stream, "cs_u_min[ {} ]={}\n", viewIdx, cs_u_min());
  fmt::print(stream, "cs_u_max[ {} ]={}\n", viewIdx, cs_u_max());
  fmt::print(stream, "cs_v_min[ {} ]={}\n", viewIdx, cs_v_min());
  fmt::print(stream, "cs_v_max[ {} ]={}\n", viewIdx, cs_v_max());
  return stream;
}

auto ChromaScaling::decodeFrom(Common::InputBitstream &bitstream, const MivViewParamsList &mvpl)
    -> ChromaScaling {
  auto x = ChromaScaling{};

  const auto bitDepth = mvpl.mvp_chroma_scaling_bit_depth_minus1() + 1U;

  x.cs_u_min(bitstream.readBits<uint16_t>(bitDepth));
  x.cs_u_max(bitstream.readBits<uint16_t>(bitDepth));
  x.cs_v_min(bitstream.readBits<uint16_t>(bitDepth));
  x.cs_v_max(bitstream.readBits<uint16_t>(bitDepth));

  return x;
}

void ChromaScaling::encodeTo(Common::OutputBitstream &bitstream,
                             const MivViewParamsList &mvpl) const {
  const auto bitDepth = mvpl.mvp_chroma_scaling_bit_depth_minus1() + 1U;

  bitstream.writeBits(cs_u_min(), bitDepth);
  bitstream.writeBits(cs_u_max(), bitDepth);
  bitstream.writeBits(cs_v_min(), bitDepth);
  bitstream.writeBits(cs_v_max(), bitDepth);
}

auto CameraExtrinsics::printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream & {
  fmt::print(stream, "ce_view_pos_x[ {} ]={}\n", viewIdx, ce_view_pos_x());
  fmt::print(stream, "ce_view_pos_y[ {} ]={}\n", viewIdx, ce_view_pos_y());
  fmt::print(stream, "ce_view_pos_z[ {} ]={}\n", viewIdx, ce_view_pos_z());
  fmt::print(stream, "ce_view_quat_x[ {} ]={}\n", viewIdx, ce_view_quat_x());
  fmt::print(stream, "ce_view_quat_y[ {} ]={}\n", viewIdx, ce_view_quat_y());
  fmt::print(stream, "ce_view_quat_z[ {} ]={}\n", viewIdx, ce_view_quat_z());
  return stream;
}

auto CameraExtrinsics::decodeFrom(Common::InputBitstream &bitstream) -> CameraExtrinsics {
  auto x = CameraExtrinsics{};

  x.ce_view_pos_x(bitstream.getFloat32());
  x.ce_view_pos_y(bitstream.getFloat32());
  x.ce_view_pos_z(bitstream.getFloat32());

  x.ce_view_quat_x(bitstream.getInt32());
  x.ce_view_quat_y(bitstream.getInt32());
  x.ce_view_quat_z(bitstream.getInt32());

  return x;
}

void CameraExtrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFloat32(ce_view_pos_x());
  bitstream.putFloat32(ce_view_pos_y());
  bitstream.putFloat32(ce_view_pos_z());

  bitstream.putInt32(ce_view_quat_x());
  bitstream.putInt32(ce_view_quat_y());
  bitstream.putInt32(ce_view_quat_z());
}

auto DepthQuantization::printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream & {
  VERIFY_MIVBITSTREAM(dq_quantization_law() == 0 || dq_quantization_law() == 2);

  stream << "dq_quantization_law[ " << viewIdx << " ]=" << int32_t{dq_quantization_law()}
         << "\ndq_norm_disp_low[ " << viewIdx << " ]=" << dq_norm_disp_low()
         << "\ndq_norm_disp_high[ " << viewIdx << " ]=" << dq_norm_disp_high()
         << "\ndq_depth_occ_threshold_default[ " << viewIdx
         << " ]=" << dq_depth_occ_threshold_default() << '\n';
  return stream;
}

auto DepthQuantization::decodeFrom(Common::InputBitstream &bitstream) -> DepthQuantization {
  auto x = DepthQuantization{};

  x.dq_quantization_law(bitstream.getUExpGolomb<uint8_t>());

  if (x.dq_quantization_law() == 0) {
    x.dq_norm_disp_low(bitstream.getFloat32());
    x.dq_norm_disp_high(bitstream.getFloat32());
  }

  VERIFY_MIVBITSTREAM(x.dq_quantization_law() == 0 || x.dq_quantization_law() == 2);

  if (x.dq_quantization_law() == 2) {
    x.dq_norm_disp_low(bitstream.getFloat32());
    x.dq_norm_disp_high(bitstream.getFloat32());
    x.dq_pivot_count_minus1(bitstream.getUint8());

    for (int32_t i = 0; i <= x.dq_pivot_count_minus1(); i++) {
      x.dq_pivot_norm_disp(i, bitstream.getFloat32());
    }
  }

  x.dq_depth_occ_threshold_default(bitstream.getUExpGolomb<uint32_t>());

  return x;
}

void DepthQuantization::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUExpGolomb(dq_quantization_law());

  if (dq_quantization_law() == 0) {
    bitstream.putFloat32(dq_norm_disp_low());
    bitstream.putFloat32(dq_norm_disp_high());
  }

  if (dq_quantization_law() == 2) {
    bitstream.putFloat32(dq_norm_disp_low());
    bitstream.putFloat32(dq_norm_disp_high());
    bitstream.putUint8(dq_pivot_count_minus1());

    for (int32_t i = 0; i <= dq_pivot_count_minus1(); i++) {
      bitstream.putFloat32(dq_pivot_norm_disp(i));
    }
  }

  bitstream.putUExpGolomb(dq_depth_occ_threshold_default());
}

PruningParents::PruningParents(std::vector<uint16_t> pp_parent_idx)
    : m_pp_parent_id{std::move(pp_parent_idx)} {}

auto PruningParents::pp_is_root_flag() const noexcept -> bool { return m_pp_parent_id.empty(); }

auto PruningParents::pp_num_parent_minus1() const -> uint16_t {
  VERIFY_MIVBITSTREAM(!pp_is_root_flag());
  return static_cast<uint16_t>(m_pp_parent_id.size() - 1);
}

auto PruningParents::pp_parent_idx(uint16_t i) const -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_pp_parent_id.size());
  return m_pp_parent_id[i];
}

auto PruningParents::pp_parent_idx(uint16_t i, uint16_t value) noexcept -> PruningParents & {
  PRECONDITION(i < m_pp_parent_id.size());
  m_pp_parent_id[i] = value;
  return *this;
}

auto PruningParents::printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream & {
  stream << "pp_is_root_flag[ " << viewIdx << " ]=" << std::boolalpha << pp_is_root_flag() << '\n';
  if (!pp_is_root_flag()) {
    stream << "pp_num_parent_minus1[ " << viewIdx << " ]=" << pp_num_parent_minus1() << '\n';
    for (uint16_t i = 0; i <= pp_num_parent_minus1(); ++i) {
      stream << "pp_parent_idx[ " << viewIdx << " ][ " << i << " ]=" << pp_parent_idx(i) << '\n';
    }
  }
  return stream;
}

auto PruningParents::operator==(const PruningParents &other) const noexcept -> bool {
  return m_pp_parent_id == other.m_pp_parent_id;
}

auto PruningParents::operator!=(const PruningParents &other) const noexcept -> bool {
  return !operator==(other);
}

auto PruningParents::decodeFrom(Common::InputBitstream &bitstream, uint16_t mvp_num_views_minus1)
    -> PruningParents {
  const auto pp_is_root_flag = bitstream.getFlag();
  if (pp_is_root_flag) {
    return {};
  }

  const auto pp_num_parent_minus1 = bitstream.getUVar<size_t>(mvp_num_views_minus1);
  auto x = std::vector<uint16_t>(pp_num_parent_minus1 + 1);

  for (uint16_t &i : x) {
    i = bitstream.getUVar<uint16_t>(mvp_num_views_minus1 + uint64_t{1});
  }

  return PruningParents{x};
}

void PruningParents::encodeTo(Common::OutputBitstream &bitstream,
                              uint16_t mvp_num_views_minus1) const {
  bitstream.putFlag(pp_is_root_flag());
  if (!pp_is_root_flag()) {
    bitstream.putUVar(pp_num_parent_minus1(), mvp_num_views_minus1);
    for (uint16_t i = 0; i <= pp_num_parent_minus1(); ++i) {
      bitstream.putUVar(pp_parent_idx(i), mvp_num_views_minus1 + uint64_t{1});
    }
  }
}

auto MivViewParamsList::mvp_num_views_minus1() const -> uint16_t {
  VERIFY_MIVBITSTREAM(!m_camera_extrinsics.empty());
  return static_cast<uint16_t>(m_camera_extrinsics.size() - 1);
}

auto MivViewParamsList::mvp_view_id(uint16_t viewIdx) const -> ViewId {
  VERIFY_MIVBITSTREAM(viewIdx <= mvp_num_views_minus1());
  if (mvp_explicit_view_id_flag()) {
    PRECONDITION(m_mvp_view_id.size() == mvp_num_views_minus1() + size_t{1});
    return m_mvp_view_id[viewIdx];
  }
  return ViewId{viewIdx};
}

auto MivViewParamsList::mvp_inpaint_flag(uint16_t viewIdx) const -> bool {
  return m_mvpInpaintFlag[viewIdx];
}

auto MivViewParamsList::camera_extrinsics(uint16_t viewIdx) const -> const CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(viewIdx < m_camera_extrinsics.size());
  return m_camera_extrinsics[viewIdx];
}

auto MivViewParamsList::camera_intrinsics(uint16_t viewIdx) const -> const CameraIntrinsics & {
  if (mvp_intrinsic_params_equal_flag()) {
    viewIdx = 0; // Convenience
  }
  VERIFY_MIVBITSTREAM(viewIdx < m_camera_intrinsics.size());
  return m_camera_intrinsics[viewIdx];
}

auto MivViewParamsList::chroma_scaling(uint16_t viewIdx) const -> const ChromaScaling & {
  VERIFY_MIVBITSTREAM(viewIdx < m_mvp_chroma_scaling_values.size());
  return m_mvp_chroma_scaling_values[viewIdx];
}

auto MivViewParamsList::depth_quantization(uint16_t viewIdx) const -> const DepthQuantization & {
  if (mvp_depth_quantization_params_equal_flag()) {
    viewIdx = 0; // Convenience
  }
  VERIFY_MIVBITSTREAM(viewIdx < m_depth_quantization.size());
  return m_depth_quantization[viewIdx];
}

auto MivViewParamsList::pruning_parent(uint16_t viewIdx) const -> const PruningParents & {
  VERIFY_MIVBITSTREAM(mvp_pruning_graph_params_present_flag());
  VERIFY_MIVBITSTREAM(viewIdx < m_pruning_parent.size());
  return m_pruning_parent[viewIdx];
}

auto MivViewParamsList::mvp_chroma_scaling_bit_depth_minus1() const -> uint8_t {
  VERIFY_MIVBITSTREAM(m_mvp_chroma_scaling_bit_depth_minus1.has_value());
  return *m_mvp_chroma_scaling_bit_depth_minus1;
}

auto MivViewParamsList::mvp_device_model_id(uint16_t v) const -> uint8_t {
  VERIFY_MIVBITSTREAM(v < m_mvp_device_model_id.size());
  return m_mvp_device_model_id[v];
}

auto MivViewParamsList::sensor_extrinsics(uint16_t v, uint16_t s) const
    -> const SensorExtrinsics & {
  VERIFY_MIVBITSTREAM(v < m_sensor_extrinsics.size());
  VERIFY_MIVBITSTREAM(s < m_sensor_extrinsics[v].size());
  return m_sensor_extrinsics[v][s];
}

auto MivViewParamsList::distortion_parameters(uint16_t v, uint16_t s) const
    -> const DistortionParameters & {
  VERIFY_MIVBITSTREAM(v < m_distortion_parameters.size());
  VERIFY_MIVBITSTREAM(s < m_distortion_parameters[v].size());
  return m_distortion_parameters[v][s];
}
auto MivViewParamsList::light_source_extrinsics(uint16_t v, uint16_t s) const
    -> const LightSourceExtrinsics & {
  VERIFY_MIVBITSTREAM(v < m_light_source_extrinsics.size());
  VERIFY_MIVBITSTREAM(s < m_light_source_extrinsics[v].size());
  return m_light_source_extrinsics[v][s];
}

auto MivViewParamsList::mvp_num_views_minus1(uint16_t value) -> MivViewParamsList & {
  m_camera_extrinsics.resize(value + size_t{1});
  m_mvpInpaintFlag.resize(value + size_t{1}, false);
  m_mvp_chroma_scaling_values.resize(value + size_t{1});
  m_mvp_device_model_id.resize(value + 1);
  m_sensor_extrinsics.resize(value + 1);
  m_distortion_parameters.resize(value + 1);
  m_light_source_extrinsics.resize(value + 1);
  return *this;
}

auto MivViewParamsList::mvp_explicit_view_id_flag(bool value) noexcept -> MivViewParamsList & {
  m_mvp_explicit_view_id_flag = value;
  return *this;
}

auto MivViewParamsList::mvp_view_id(uint16_t viewIdx, ViewId viewId) -> MivViewParamsList & {
  mvp_explicit_view_id_flag(true);
  if (m_mvp_view_id.size() < mvp_num_views_minus1() + size_t{1}) {
    m_mvp_view_id.resize(mvp_num_views_minus1() + size_t{1});
  }
  PRECONDITION(viewIdx <= mvp_num_views_minus1());
  m_mvp_view_id[viewIdx] = viewId;
  return *this;
}

auto MivViewParamsList::mvp_inpaint_flag(uint16_t viewIdx, bool value) -> MivViewParamsList & {
  PRECONDITION(viewIdx <= mvp_num_views_minus1());
  m_mvpInpaintFlag[viewIdx] = value;
  return *this;
}

auto MivViewParamsList::mvp_intrinsic_params_equal_flag(bool value) -> MivViewParamsList & {
  m_mvp_intrinsic_params_equal_flag = value;
  m_camera_intrinsics.resize(value ? 1U : m_camera_extrinsics.size());
  return *this;
}

auto MivViewParamsList::mvp_depth_quantization_params_equal_flag(bool value)
    -> MivViewParamsList & {
  m_mvp_depth_quantization_params_equal_flag = value;
  m_depth_quantization.resize(value ? 1U : m_camera_extrinsics.size());
  return *this;
}

auto MivViewParamsList::mvp_chroma_scaling_bit_depth_minus1(uint8_t value) -> MivViewParamsList & {
  m_mvp_chroma_scaling_bit_depth_minus1 = value;
  return *this;
}

auto MivViewParamsList::mvp_pruning_graph_params_present_flag(bool value) -> MivViewParamsList & {
  m_mvp_pruning_graph_params_present_flag = value;
  m_pruning_parent.resize(value ? m_camera_extrinsics.size() : 0U);
  return *this;
}

auto MivViewParamsList::camera_extrinsics(uint16_t viewIdx) noexcept -> CameraExtrinsics & {
  PRECONDITION(viewIdx < m_camera_extrinsics.size());
  return m_camera_extrinsics[viewIdx];
}

auto MivViewParamsList::camera_intrinsics(uint16_t viewIdx) noexcept -> CameraIntrinsics & {
  PRECONDITION(viewIdx < m_camera_intrinsics.size());
  return m_camera_intrinsics[viewIdx];
}

auto MivViewParamsList::chroma_scaling(uint16_t viewIdx) noexcept -> ChromaScaling & {
  PRECONDITION(viewIdx < m_mvp_chroma_scaling_values.size());
  return m_mvp_chroma_scaling_values[viewIdx];
}

auto MivViewParamsList::depth_quantization(uint16_t viewIdx) noexcept -> DepthQuantization & {
  PRECONDITION(viewIdx < m_depth_quantization.size());
  return m_depth_quantization[viewIdx];
}

auto MivViewParamsList::pruning_parent(uint16_t viewIdx) -> PruningParents & {
  PRECONDITION(viewIdx < m_pruning_parent.size());
  return m_pruning_parent[viewIdx];
}

auto MivViewParamsList::mvp_device_model_id(uint16_t v, uint8_t value) -> MivViewParamsList & {
  VERIFY_MIVBITSTREAM(v < m_mvp_device_model_id.size());
  m_mvp_device_model_id[v] = value;
  return *this;
}

auto MivViewParamsList::sensor_extrinsics(uint16_t v, uint16_t s) -> SensorExtrinsics & {
  VERIFY_MIVBITSTREAM(v < m_sensor_extrinsics.size());
  if (s >= m_sensor_extrinsics[v].size()) {
    m_sensor_extrinsics[v].resize(s + 1);
  }
  return m_sensor_extrinsics[v][s];
}

auto MivViewParamsList::distortion_parameters(uint16_t v, uint16_t s) -> DistortionParameters & {
  VERIFY_MIVBITSTREAM(v < m_distortion_parameters.size());
  if (s >= m_distortion_parameters[v].size()) {
    m_distortion_parameters[v].resize(s + 1);
  }
  return m_distortion_parameters[v][s];
}

auto MivViewParamsList::light_source_extrinsics(uint16_t v, uint16_t s) -> LightSourceExtrinsics & {
  VERIFY_MIVBITSTREAM(v < m_light_source_extrinsics.size());
  if (s >= m_light_source_extrinsics[v].size()) {
    m_light_source_extrinsics[v].resize(s + 1);
  }
  return m_light_source_extrinsics[v][s];
}

auto MivViewParamsList::printTo(std::ostream &stream,
                                const CommonAtlasSequenceParameterSetRBSP &casps) const
    -> std::ostream & {
  fmt::print(stream, "mvp_num_views_minus1={}\n", mvp_num_views_minus1());
  fmt::print(stream, "mvp_explicit_view_id_flag={}\n", mvp_explicit_view_id_flag());

  if (mvp_explicit_view_id_flag()) {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      fmt::print(stream, "mvp_view_id[ {} ]={}\n", v, mvp_view_id(v));
    }
  }

  CaptureDeviceInformation::Semantics cdi_semantics;
  if (casps.casps_miv_2_extension_present_flag() &&
      casps.casps_miv_2_extension().casme_capture_device_information_present_flag()) {
    casps.casps_miv_2_extension().capture_device_information().applySemantics(cdi_semantics);
  }

  for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
    camera_extrinsics(v).printTo(stream, v);
    fmt::print(stream, "mvp_inpaint_flag[ {} ]={}\n", v, mvp_inpaint_flag(v));

    if (casps.casps_miv_2_extension_present_flag() &&
        casps.casps_miv_2_extension().casme_capture_device_information_present_flag()) {
      fmt::print(stream, "mvp_device_model_id[ {} ]={}\n", v, mvp_device_model_id(v));
      auto i = mvp_device_model_id(v);
      for (uint16_t s = 0; s < cdi_semantics.sensorCount[i]; s++) {
        if (cdi_semantics.intraSensorParallaxFlag[i]) {
          sensor_extrinsics(v, s).printTo(stream, v, s);
        }
        distortion_parameters(v, s).printTo(stream, v, s);
      }
      for (uint16_t s = 0; s < cdi_semantics.lightSourceCount[i]; s++) {
        light_source_extrinsics(v, s).printTo(stream, v, s);
      }
    }
  }

  fmt::print(stream, "mvp_intrinsic_params_equal_flag={}\n", mvp_intrinsic_params_equal_flag());

  if (mvp_intrinsic_params_equal_flag()) {
    camera_intrinsics(0).printTo(stream, 0);
  } else {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      camera_intrinsics(v).printTo(stream, v);
    }
  }

  if (m_mvp_depth_quantization_params_equal_flag) {
    fmt::print(stream, "mvp_depth_quantization_params_equal_flag={}\n",
               mvp_depth_quantization_params_equal_flag());

    if (mvp_depth_quantization_params_equal_flag()) {
      depth_quantization(0).printTo(stream, 0);
    } else {
      for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
        depth_quantization(v).printTo(stream, v);
      }
    }
  }
  fmt::print(stream, "mvp_pruning_graph_params_present_flag={}\n",
             mvp_pruning_graph_params_present_flag());

  if (mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      pruning_parent(v).printTo(stream, v);
    }
  }

  if (m_mvp_depth_reprojection_flag) {
    fmt::print(stream, "mvp_depth_reprojection_flag={}\n", mvp_depth_reprojection_flag());
  }

  for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
    chroma_scaling(v).printTo(stream, v);
  }

  return stream;
}

auto MivViewParamsList::operator==(const MivViewParamsList &other) const noexcept -> bool {
  bool result =
      m_mvp_explicit_view_id_flag == other.m_mvp_explicit_view_id_flag &&
      m_mvp_view_id == other.m_mvp_view_id && m_camera_extrinsics == other.m_camera_extrinsics &&
      m_mvpInpaintFlag == other.m_mvpInpaintFlag &&
      m_mvp_intrinsic_params_equal_flag == other.m_mvp_intrinsic_params_equal_flag &&
      m_camera_intrinsics == other.m_camera_intrinsics &&
      m_mvp_depth_quantization_params_equal_flag ==
          other.m_mvp_depth_quantization_params_equal_flag &&
      m_depth_quantization == other.m_depth_quantization &&
      m_mvp_pruning_graph_params_present_flag == other.m_mvp_pruning_graph_params_present_flag &&
      m_pruning_parent == other.m_pruning_parent &&
      m_mvp_chroma_scaling_values == other.m_mvp_chroma_scaling_values &&
      m_mvp_depth_reprojection_flag == other.m_mvp_depth_reprojection_flag &&
      m_mvp_chroma_scaling_bit_depth_minus1 == other.m_mvp_chroma_scaling_bit_depth_minus1 &&
      m_mvp_device_model_id == other.m_mvp_device_model_id;

  for (uint16_t i = 0; i <= mvp_num_views_minus1() && result; i++) {
    result = m_sensor_extrinsics[i] == other.m_sensor_extrinsics[i] &&
             m_distortion_parameters[i] == other.m_distortion_parameters[i] &&
             m_light_source_extrinsics[i] == other.m_light_source_extrinsics[i];
  }

  return result;
}

auto MivViewParamsList::operator!=(const MivViewParamsList &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsList::decodeFrom(Common::InputBitstream &bitstream,
                                   const CommonAtlasSequenceParameterSetRBSP &casps)
    -> MivViewParamsList {
  auto x = MivViewParamsList{};

  x.mvp_num_views_minus1(bitstream.getUint16());
  x.mvp_explicit_view_id_flag(bitstream.getFlag());

  if (x.mvp_explicit_view_id_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.mvp_view_id(v, ViewId::decodeFrom(bitstream, 16));
    }
  }

  CaptureDeviceInformation::Semantics cdi_semantics;
  if (casps.casps_miv_2_extension_present_flag() &&
      casps.casps_miv_2_extension().casme_capture_device_information_present_flag()) {
    casps.casps_miv_2_extension().capture_device_information().applySemantics(cdi_semantics);
  }
  for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
    x.camera_extrinsics(v) = CameraExtrinsics::decodeFrom(bitstream);
    x.mvp_inpaint_flag(v, bitstream.getFlag());

    if (casps.casps_miv_2_extension_present_flag() &&
        casps.casps_miv_2_extension().casme_capture_device_information_present_flag()) {
      x.mvp_device_model_id(v, bitstream.readBits<uint8_t>(6));
      auto i = x.mvp_device_model_id(v);
      for (uint16_t s = 0; s < cdi_semantics.sensorCount[i]; s++) {
        if (cdi_semantics.intraSensorParallaxFlag[i]) {
          x.sensor_extrinsics(v, s) = SensorExtrinsics::decodeFrom(bitstream);
        }
        x.distortion_parameters(v, s) = DistortionParameters::decodeFrom(bitstream);
      }
      for (uint16_t s = 0; s < cdi_semantics.lightSourceCount[i]; s++) {
        x.light_source_extrinsics(v, s) = LightSourceExtrinsics::decodeFrom(bitstream);
      }
    }
  }

  x.mvp_intrinsic_params_equal_flag(bitstream.getFlag());

  if (x.mvp_intrinsic_params_equal_flag()) {
    x.camera_intrinsics() = CameraIntrinsics::decodeFrom(bitstream);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.camera_intrinsics(v) = CameraIntrinsics::decodeFrom(bitstream);
    }
  }

  if (casps.casps_miv_extension().casme_depth_quantization_params_present_flag()) {
    x.mvp_depth_quantization_params_equal_flag(bitstream.getFlag());

    if (x.mvp_depth_quantization_params_equal_flag()) {
      x.depth_quantization() = DepthQuantization::decodeFrom(bitstream);
    } else {
      for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
        x.depth_quantization(v) = DepthQuantization::decodeFrom(bitstream);
      }
    }
  }

  x.mvp_pruning_graph_params_present_flag(bitstream.getFlag());

  if (x.mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.pruning_parent(v) = PruningParents::decodeFrom(bitstream, x.mvp_num_views_minus1());
    }
  }

  if (casps.casps_miv_2_extension_present_flag() &&
      casps.casps_miv_2_extension().casme_decoder_side_depth_estimation_flag()) {
    x.mvp_depth_reprojection_flag(bitstream.getFlag());
  }

  if (casps.casps_miv_2_extension_present_flag() &&
      casps.casps_miv_2_extension().casme_chroma_scaling_present_flag()) {
    x.mvp_chroma_scaling_bit_depth_minus1(bitstream.readBits<uint8_t>(5));

    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.chroma_scaling(v) = ChromaScaling::decodeFrom(bitstream, x);
    }
  }

  return x;
}

void MivViewParamsList::encodeTo(Common::OutputBitstream &bitstream,
                                 const CommonAtlasSequenceParameterSetRBSP &casps) const {
  bitstream.putUint16(mvp_num_views_minus1());
  bitstream.putFlag(mvp_explicit_view_id_flag());

  if (mvp_explicit_view_id_flag()) {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      mvp_view_id(v).encodeTo(bitstream, 16);
    }
  }

  CaptureDeviceInformation::Semantics cdi_semantics;
  if (casps.casps_miv_2_extension_present_flag() &&
      casps.casps_miv_2_extension().casme_capture_device_information_present_flag()) {
    casps.casps_miv_2_extension().capture_device_information().applySemantics(cdi_semantics);
  }
  for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
    camera_extrinsics(v).encodeTo(bitstream);
    bitstream.putFlag(mvp_inpaint_flag(v));

    if (casps.casps_miv_2_extension_present_flag() &&
        casps.casps_miv_2_extension().casme_capture_device_information_present_flag()) {
      auto i = mvp_device_model_id(v);
      bitstream.writeBits<uint8_t>(i, 6);
      for (uint16_t s = 0; s < cdi_semantics.sensorCount[i]; s++) {
        if (cdi_semantics.intraSensorParallaxFlag[i]) {
          sensor_extrinsics(v, s).encodeTo(bitstream);
        }
        distortion_parameters(v, s).encodeTo(bitstream);
      }
      for (uint16_t s = 0; s < cdi_semantics.lightSourceCount[i]; s++) {
        light_source_extrinsics(v, s).encodeTo(bitstream);
      }
    }
  }

  bitstream.putFlag(mvp_intrinsic_params_equal_flag());

  if (mvp_intrinsic_params_equal_flag()) {
    camera_intrinsics().encodeTo(bitstream);
  } else {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      camera_intrinsics(v).encodeTo(bitstream);
    }
  }

  if (casps.casps_miv_extension().casme_depth_quantization_params_present_flag()) {
    bitstream.putFlag(mvp_depth_quantization_params_equal_flag());

    if (mvp_depth_quantization_params_equal_flag()) {
      depth_quantization().encodeTo(bitstream);
    } else {
      for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
        depth_quantization(v).encodeTo(bitstream);
      }
    }
  }
  bitstream.putFlag(mvp_pruning_graph_params_present_flag());

  if (mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      pruning_parent(v).encodeTo(bitstream, mvp_num_views_minus1());
    }
  }

  if (casps.casps_miv_2_extension_present_flag() &&
      casps.casps_miv_2_extension().casme_decoder_side_depth_estimation_flag()) {
    bitstream.putFlag(mvp_depth_reprojection_flag());
  }

  if (casps.casps_miv_2_extension_present_flag() &&
      casps.casps_miv_2_extension().casme_chroma_scaling_present_flag()) {
    bitstream.writeBits(mvp_chroma_scaling_bit_depth_minus1(), 5);

    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      chroma_scaling(v).encodeTo(bitstream, *this);
    }
  }
}

auto CafMivExtension::came_update_extrinsics_flag() const -> bool {
  VERIFY_MIVBITSTREAM(m_came_update_extrinsics_flag.has_value());
  return *m_came_update_extrinsics_flag;
}

auto CafMivExtension::came_update_intrinsics_flag() const -> bool {
  VERIFY_MIVBITSTREAM(m_came_update_intrinsics_flag.has_value());
  return *m_came_update_intrinsics_flag;
}

auto CafMivExtension::came_update_depth_quantization_flag() const -> bool {
  return m_came_update_depth_quantization_flag.value_or(false);
}

auto CafMivExtension::came_update_chroma_scaling_flag() const -> bool {
  return m_came_update_chroma_scaling_flag.value_or(false);
}

auto CafMivExtension::came_update_sensor_extrinsics_flag() const -> bool {
  return m_came_update_sensor_extrinsics_flag.value_or(false);
}

auto CafMivExtension::came_update_distortion_parameters_flag() const -> bool {
  return m_came_update_distortion_parameters_flag.value_or(false);
}

auto CafMivExtension::came_update_light_source_extrinsics_flag() const -> bool {
  return m_came_update_light_source_extrinsics_flag.value_or(false);
}

auto CafMivExtension::miv_view_params_list() const -> const MivViewParamsList & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_list.has_value());
  return *m_miv_view_params_list;
}

auto CafMivExtension::miv_view_params_update_extrinsics() const
    -> const MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(came_update_extrinsics_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_extrinsics.has_value());
  return *m_miv_view_params_update_extrinsics;
}

auto CafMivExtension::miv_view_params_update_intrinsics() const
    -> const MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(came_update_intrinsics_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_intrinsics.has_value());
  return *m_miv_view_params_update_intrinsics;
}

auto CafMivExtension::miv_view_params_update_depth_quantization() const
    -> const MivViewParamsUpdateDepthQuantization & {
  VERIFY_MIVBITSTREAM(came_update_depth_quantization_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_depth_quantization.has_value());
  return *m_miv_view_params_update_depth_quantization;
}

auto CafMivExtension::miv_view_params_update_chroma_scaling() const
    -> const MivViewParamsUpdateChromaScaling & {
  VERIFY_MIVBITSTREAM(came_update_chroma_scaling_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_chroma_scaling.has_value());
  return *m_miv_view_params_update_chroma_scaling;
}

auto CafMivExtension::miv_view_params_update_sensor_extrinsics() const
    -> const MivViewParamsUpdateSensorExtrinsics & {
  VERIFY_MIVBITSTREAM(came_update_sensor_extrinsics_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_sensor_extrinsics.has_value());
  return *m_miv_view_params_update_sensor_extrinsics;
}

auto CafMivExtension::miv_view_params_update_distortion_parameters() const
    -> const MivViewParamsUpdateDistortionParameters & {
  VERIFY_MIVBITSTREAM(came_update_distortion_parameters_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_distortion_parameters.has_value());
  return *m_miv_view_params_update_distortion_parameters;
}

auto CafMivExtension::miv_view_params_update_light_source_extrinsics() const
    -> const MivViewParamsUpdateLightSourceExtrinsics & {
  VERIFY_MIVBITSTREAM(came_update_light_source_extrinsics_flag());
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_light_source_extrinsics.has_value());
  return *m_miv_view_params_update_light_source_extrinsics;
}

auto CafMivExtension::miv_view_params_list() noexcept -> MivViewParamsList & {
  if (!m_miv_view_params_list) {
    m_miv_view_params_list = MivViewParamsList{};
  }
  return *m_miv_view_params_list;
}

auto CafMivExtension::miv_view_params_update_extrinsics() noexcept
    -> MivViewParamsUpdateExtrinsics & {
  came_update_extrinsics_flag(true);
  if (!m_miv_view_params_update_extrinsics) {
    m_miv_view_params_update_extrinsics = MivViewParamsUpdateExtrinsics{};
  }
  return *m_miv_view_params_update_extrinsics;
}

auto CafMivExtension::miv_view_params_update_intrinsics() noexcept
    -> MivViewParamsUpdateIntrinsics & {
  came_update_intrinsics_flag(true);
  if (!m_miv_view_params_update_intrinsics) {
    m_miv_view_params_update_intrinsics = MivViewParamsUpdateIntrinsics{};
  }
  return *m_miv_view_params_update_intrinsics;
}

auto CafMivExtension::miv_view_params_update_depth_quantization() noexcept
    -> MivViewParamsUpdateDepthQuantization & {
  came_update_depth_quantization_flag(true);
  if (!m_miv_view_params_update_depth_quantization) {
    m_miv_view_params_update_depth_quantization = MivViewParamsUpdateDepthQuantization{};
  }
  return *m_miv_view_params_update_depth_quantization;
}

auto CafMivExtension::miv_view_params_update_chroma_scaling() noexcept
    -> MivViewParamsUpdateChromaScaling & {
  came_update_chroma_scaling_flag(true);
  if (!m_miv_view_params_update_chroma_scaling) {
    m_miv_view_params_update_chroma_scaling = MivViewParamsUpdateChromaScaling{};
  }
  return *m_miv_view_params_update_chroma_scaling;
}

auto CafMivExtension::miv_view_params_update_sensor_extrinsics() noexcept
    -> MivViewParamsUpdateSensorExtrinsics & {
  came_update_sensor_extrinsics_flag(true);
  if (!m_miv_view_params_update_sensor_extrinsics) {
    m_miv_view_params_update_sensor_extrinsics = MivViewParamsUpdateSensorExtrinsics{};
  }
  return *m_miv_view_params_update_sensor_extrinsics;
}

auto CafMivExtension::miv_view_params_update_distortion_parameters() noexcept
    -> MivViewParamsUpdateDistortionParameters & {
  came_update_distortion_parameters_flag(true);
  if (!m_miv_view_params_update_distortion_parameters) {
    m_miv_view_params_update_distortion_parameters = MivViewParamsUpdateDistortionParameters{};
  }
  return *m_miv_view_params_update_distortion_parameters;
}

auto CafMivExtension::miv_view_params_update_light_source_extrinsics() noexcept
    -> MivViewParamsUpdateLightSourceExtrinsics & {
  came_update_light_source_extrinsics_flag(true);
  if (!m_miv_view_params_update_light_source_extrinsics) {
    m_miv_view_params_update_light_source_extrinsics = MivViewParamsUpdateLightSourceExtrinsics{};
  }
  return *m_miv_view_params_update_light_source_extrinsics;
}

auto CafMivExtension::came_update_extrinsics_flag(bool value) noexcept -> CafMivExtension & {
  m_came_update_extrinsics_flag = value;
  return *this;
}

auto CafMivExtension::came_update_intrinsics_flag(bool value) noexcept -> CafMivExtension & {
  m_came_update_intrinsics_flag = value;
  return *this;
}

auto CafMivExtension::came_update_depth_quantization_flag(bool value) noexcept
    -> CafMivExtension & {
  m_came_update_depth_quantization_flag = value;
  return *this;
}

auto CafMivExtension::came_update_chroma_scaling_flag(bool value) noexcept -> CafMivExtension & {
  m_came_update_chroma_scaling_flag = value;
  return *this;
}

auto CafMivExtension::came_update_sensor_extrinsics_flag(bool value) noexcept -> CafMivExtension & {
  m_came_update_sensor_extrinsics_flag = value;
  m_came_update_distortion_parameters_flag =
      m_came_update_distortion_parameters_flag.value_or(false);
  m_came_update_light_source_extrinsics_flag =
      m_came_update_light_source_extrinsics_flag.value_or(false);
  return *this;
}

auto CafMivExtension::came_update_distortion_parameters_flag(bool value) noexcept
    -> CafMivExtension & {
  m_came_update_sensor_extrinsics_flag = m_came_update_sensor_extrinsics_flag.value_or(false);
  m_came_update_distortion_parameters_flag = value;
  m_came_update_light_source_extrinsics_flag =
      m_came_update_light_source_extrinsics_flag.value_or(false);
  return *this;
}

auto CafMivExtension::came_update_light_source_extrinsics_flag(bool value) noexcept
    -> CafMivExtension & {
  m_came_update_sensor_extrinsics_flag = m_came_update_sensor_extrinsics_flag.value_or(false);
  m_came_update_distortion_parameters_flag =
      m_came_update_distortion_parameters_flag.value_or(false);
  m_came_update_light_source_extrinsics_flag = value;
  return *this;
}

auto CafMivExtension::printTo(std::ostream &stream,
                              const CommonAtlasSequenceParameterSetRBSP &casps) const
    -> std::ostream & {
  if (m_miv_view_params_list) {
    fmt::print(stream, "miv_view_params_list=");
    miv_view_params_list().printTo(stream, casps);
  } else {
    fmt::print(stream, "came_update_extrinsics_flag={}\n", came_update_extrinsics_flag());
    fmt::print(stream, "came_update_intrinsics_flag={}\n", came_update_intrinsics_flag());
    if (m_came_update_depth_quantization_flag) {
      fmt::print(stream, "came_update_depth_quantization_flag={}\n",
                 came_update_depth_quantization_flag());
    }
    if (m_came_update_chroma_scaling_flag) {
      fmt::print(stream, "came_update_chroma_scaling_flag={}\n", came_update_chroma_scaling_flag());
    }
    if (m_came_update_sensor_extrinsics_flag) {
      fmt::print(stream, "came_update_sensor_extrinsics_flag={}\n",
                 came_update_sensor_extrinsics_flag());
    }
    if (m_came_update_distortion_parameters_flag) {
      fmt::print(stream, "came_update_distortion_parameters_flag={}\n",
                 came_update_distortion_parameters_flag());
    }
    if (m_came_update_light_source_extrinsics_flag) {
      fmt::print(stream, "came_update_light_source_extrinsics_flag={}\n",
                 came_update_light_source_extrinsics_flag());
    }
    if (came_update_extrinsics_flag()) {
      stream << miv_view_params_update_extrinsics();
    }
    if (came_update_intrinsics_flag()) {
      stream << miv_view_params_update_intrinsics();
    }
    if (m_came_update_depth_quantization_flag.value_or(false)) {
      stream << miv_view_params_update_depth_quantization();
    }
    if (m_came_update_chroma_scaling_flag.value_or(false)) {
      stream << miv_view_params_update_chroma_scaling();
    }
    if (m_came_update_sensor_extrinsics_flag.value_or(false)) {
      stream << miv_view_params_update_sensor_extrinsics();
    }
    if (m_came_update_distortion_parameters_flag.value_or(false)) {
      stream << miv_view_params_update_distortion_parameters();
    }
    if (m_came_update_light_source_extrinsics_flag.value_or(false)) {
      stream << miv_view_params_update_light_source_extrinsics();
    }
  }
  return stream;
}

auto CafMivExtension::operator==(const CafMivExtension &other) const -> bool {
  if (m_miv_view_params_list != other.m_miv_view_params_list) {
    return false;
  }
  if (m_miv_view_params_list.has_value()) {
    return true;
  }
  if (came_update_extrinsics_flag() != other.came_update_extrinsics_flag() ||
      came_update_intrinsics_flag() != other.came_update_intrinsics_flag() ||
      m_came_update_depth_quantization_flag != other.m_came_update_depth_quantization_flag ||
      m_came_update_chroma_scaling_flag != other.m_came_update_chroma_scaling_flag ||
      m_came_update_sensor_extrinsics_flag != other.m_came_update_sensor_extrinsics_flag ||
      m_came_update_distortion_parameters_flag != other.m_came_update_distortion_parameters_flag ||
      m_came_update_light_source_extrinsics_flag !=
          other.m_came_update_light_source_extrinsics_flag) {
    return false;
  }
  if (came_update_extrinsics_flag() &&
      miv_view_params_update_extrinsics() != other.miv_view_params_update_extrinsics()) {
    return false;
  }
  if (came_update_intrinsics_flag() &&
      miv_view_params_update_intrinsics() != other.miv_view_params_update_intrinsics()) {
    return false;
  }
  if (m_came_update_depth_quantization_flag.value_or(false) &&
      miv_view_params_update_depth_quantization() !=
          other.miv_view_params_update_depth_quantization()) {
    return false;
  }
  if (m_came_update_chroma_scaling_flag.value_or(false) &&
      miv_view_params_update_chroma_scaling() != other.miv_view_params_update_chroma_scaling()) {
    return false;
  }
  if (m_came_update_sensor_extrinsics_flag.value_or(false) &&
      miv_view_params_update_sensor_extrinsics() !=
          other.miv_view_params_update_sensor_extrinsics()) {
    return false;
  }
  if (m_came_update_distortion_parameters_flag.value_or(false) &&
      miv_view_params_update_distortion_parameters() !=
          other.miv_view_params_update_distortion_parameters()) {
    return false;
  }
  if (m_came_update_light_source_extrinsics_flag.value_or(false) &&
      miv_view_params_update_light_source_extrinsics() !=
          other.miv_view_params_update_light_source_extrinsics()) {
    return false;
  }

  return true;
}

auto CafMivExtension::operator!=(const CafMivExtension &other) const -> bool {
  return !operator==(other);
}

auto CafMivExtension::decodeFrom(Common::InputBitstream &bitstream, const NalUnitHeader &nuh,
                                 const CommonAtlasSequenceParameterSetRBSP &casps)
    -> CafMivExtension {
  auto x = CafMivExtension{};

  if (nuh.nal_unit_type() == NalUnitType::NAL_CAF_IDR) {
    x.miv_view_params_list() = MivViewParamsList::decodeFrom(bitstream, casps);
  } else {
    x.came_update_extrinsics_flag(bitstream.getFlag());
    x.came_update_intrinsics_flag(bitstream.getFlag());

    if (casps.casps_miv_extension().casme_depth_quantization_params_present_flag()) {
      x.came_update_depth_quantization_flag(bitstream.getFlag());
    }
    if (casps.casps_miv_2_extension_present_flag() &&
        casps.casps_miv_2_extension().casme_chroma_scaling_present_flag()) {
      x.came_update_chroma_scaling_flag(bitstream.getFlag());
    }
    if (casps.casps_miv_2_extension_present_flag() &&
        casps.casps_miv_2_extension().casme_capture_device_information_present_flag()) {
      x.came_update_sensor_extrinsics_flag(bitstream.getFlag());
      x.came_update_distortion_parameters_flag(bitstream.getFlag());
      x.came_update_light_source_extrinsics_flag(bitstream.getFlag());
    }

    if (x.came_update_extrinsics_flag()) {
      x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
    }
    if (x.came_update_intrinsics_flag()) {
      x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
    }
    if (x.came_update_depth_quantization_flag()) {
      x.miv_view_params_update_depth_quantization() =
          MivViewParamsUpdateDepthQuantization::decodeFrom(bitstream);
    }
    if (x.came_update_chroma_scaling_flag()) {
      NOT_IMPLEMENTED;
    }
    if (x.came_update_sensor_extrinsics_flag()) {
      x.miv_view_params_update_sensor_extrinsics() =
          MivViewParamsUpdateSensorExtrinsics::decodeFrom(bitstream);
    }
    if (x.came_update_distortion_parameters_flag()) {
      x.miv_view_params_update_distortion_parameters() =
          MivViewParamsUpdateDistortionParameters::decodeFrom(bitstream);
    }
    if (x.came_update_light_source_extrinsics_flag()) {
      x.miv_view_params_update_light_source_extrinsics() =
          MivViewParamsUpdateLightSourceExtrinsics::decodeFrom(bitstream);
    }
  }

  return x;
}

void CafMivExtension::encodeTo(Common::OutputBitstream &bitstream, const NalUnitHeader &nuh,
                               const CommonAtlasSequenceParameterSetRBSP &casps) const {
  if (nuh.nal_unit_type() == NalUnitType::NAL_CAF_IDR) {
    miv_view_params_list().encodeTo(bitstream, casps);
  } else {
    bitstream.putFlag(came_update_extrinsics_flag());
    bitstream.putFlag(came_update_intrinsics_flag());

    if (casps.casps_miv_extension().casme_depth_quantization_params_present_flag()) {
      bitstream.putFlag(came_update_depth_quantization_flag());
    }
    if (casps.casps_miv_2_extension_present_flag() &&
        casps.casps_miv_2_extension().casme_chroma_scaling_present_flag()) {
      bitstream.putFlag(came_update_chroma_scaling_flag());
    }
    if (casps.casps_miv_2_extension_present_flag() &&
        casps.casps_miv_2_extension().casme_capture_device_information_present_flag()) {
      bitstream.putFlag(came_update_sensor_extrinsics_flag());
      bitstream.putFlag(came_update_distortion_parameters_flag());
      bitstream.putFlag(came_update_light_source_extrinsics_flag());
    }

    if (came_update_extrinsics_flag()) {
      miv_view_params_update_extrinsics().encodeTo(bitstream);
    }
    if (came_update_intrinsics_flag()) {
      miv_view_params_update_intrinsics().encodeTo(bitstream);
    }
    if (came_update_depth_quantization_flag()) {
      miv_view_params_update_depth_quantization().encodeTo(bitstream);
    }
    if (came_update_chroma_scaling_flag()) {
      NOT_IMPLEMENTED;
    }
    if (came_update_sensor_extrinsics_flag()) {
      miv_view_params_update_sensor_extrinsics().encodeTo(bitstream);
    }
    if (came_update_distortion_parameters_flag()) {
      miv_view_params_update_distortion_parameters().encodeTo(bitstream);
    }
    if (came_update_light_source_extrinsics_flag()) {
      miv_view_params_update_light_source_extrinsics().encodeTo(bitstream);
    }
  }
}

auto MivViewParamsList::mvp_depth_quantization_params_equal_flag() const -> bool {
  VERIFY_MIVBITSTREAM(m_mvp_depth_quantization_params_equal_flag.has_value());
  return *m_mvp_depth_quantization_params_equal_flag;
}

auto MivViewParamsUpdateExtrinsics::mvpue_num_view_updates_minus1() const noexcept -> uint16_t {
  return m_mvpue_num_view_updates_minus1;
}

auto MivViewParamsUpdateExtrinsics::mvpue_view_idx(const uint16_t i) const -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpue_view_idx.size());
  return m_mvpue_view_idx[i];
}

auto MivViewParamsUpdateExtrinsics::camera_extrinsics(const uint16_t i) const
    -> const CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_extrinsics.size());
  return m_camera_extrinsics[i];
}
auto MivViewParamsUpdateExtrinsics::camera_extrinsics(const uint16_t i) noexcept
    -> CameraExtrinsics & {
  PRECONDITION(i < m_camera_extrinsics.size());
  return m_camera_extrinsics[i];
}

auto MivViewParamsUpdateExtrinsics::mvpue_num_view_updates_minus1(const uint16_t value)
    -> MivViewParamsUpdateExtrinsics & {
  m_mvpue_num_view_updates_minus1 = value;
  m_mvpue_view_idx.resize(value + size_t{1});
  m_camera_extrinsics.resize(value + size_t{1});
  return *this;
}
auto MivViewParamsUpdateExtrinsics::mvpue_view_idx(const uint16_t i, const uint16_t value) noexcept
    -> MivViewParamsUpdateExtrinsics & {
  PRECONDITION(i < m_camera_extrinsics.size());
  m_mvpue_view_idx[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const MivViewParamsUpdateExtrinsics &x) -> std::ostream & {
  stream << "mvpue_num_view_updates_minus1=" << x.mvpue_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpue_num_view_updates_minus1(); ++i) {
    stream << "mvpue_view_idx[ " << i << " ]=" << x.mvpue_view_idx(i) << '\n';
    x.camera_extrinsics(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateExtrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpue_num_view_updates_minus1());
  for (uint16_t i = 0; i <= mvpue_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpue_view_idx(i));
    camera_extrinsics(i).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateExtrinsics::decodeFrom(Common::InputBitstream &bitstream)
    -> MivViewParamsUpdateExtrinsics {
  auto x = MivViewParamsUpdateExtrinsics{};
  x.mvpue_num_view_updates_minus1(bitstream.getUint16());
  for (uint16_t i = 0; i <= x.mvpue_num_view_updates_minus1(); ++i) {
    x.mvpue_view_idx(i, bitstream.getUint16());
    x.camera_extrinsics(i) = CameraExtrinsics::decodeFrom(bitstream);
  }
  return x;
}

auto MivViewParamsUpdateExtrinsics::operator==(
    const MivViewParamsUpdateExtrinsics &other) const noexcept -> bool {
  return m_mvpue_num_view_updates_minus1 == other.m_mvpue_num_view_updates_minus1 &&
         m_mvpue_view_idx == other.m_mvpue_view_idx &&
         m_camera_extrinsics == other.m_camera_extrinsics;
}

auto MivViewParamsUpdateExtrinsics::operator!=(
    const MivViewParamsUpdateExtrinsics &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsUpdateIntrinsics::mvpui_num_view_updates_minus1() const noexcept -> uint16_t {
  return m_mvpui_num_view_updates_minus1;
}

auto MivViewParamsUpdateIntrinsics::mvpui_view_idx(const uint16_t i) const -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpui_view_idx.size());
  return m_mvpui_view_idx[i];
}

auto MivViewParamsUpdateIntrinsics::camera_intrinsics(const uint16_t i) const
    -> const CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_intrinsics.size());
  return m_camera_intrinsics[i];
}

auto MivViewParamsUpdateIntrinsics::camera_intrinsics(const uint16_t i) noexcept
    -> CameraIntrinsics & {
  PRECONDITION(i < m_camera_intrinsics.size());
  return m_camera_intrinsics[i];
}

auto MivViewParamsUpdateIntrinsics::mvpui_num_view_updates_minus1(const uint16_t value)
    -> MivViewParamsUpdateIntrinsics & {
  m_mvpui_num_view_updates_minus1 = value;
  m_mvpui_view_idx.resize(m_mvpui_num_view_updates_minus1 + size_t{1});
  m_camera_intrinsics.resize(m_mvpui_num_view_updates_minus1 + size_t{1});
  return *this;
}

auto MivViewParamsUpdateIntrinsics::mvpui_view_idx(const uint16_t i, const uint16_t value) noexcept
    -> MivViewParamsUpdateIntrinsics & {
  PRECONDITION(i < m_mvpui_view_idx.size());
  m_mvpui_view_idx[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const MivViewParamsUpdateIntrinsics &x) -> std::ostream & {
  stream << "mvpui_num_view_updates_minus1=" << x.mvpui_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpui_num_view_updates_minus1(); ++i) {
    stream << "mvpui_view_idx[ " << i << " ]=" << x.mvpui_view_idx(i) << '\n';
    x.camera_intrinsics(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateIntrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpui_num_view_updates_minus1());
  for (uint16_t i = 0; i <= mvpui_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpui_view_idx(i));
    camera_intrinsics(i).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateIntrinsics::decodeFrom(Common::InputBitstream &bitstream)
    -> MivViewParamsUpdateIntrinsics {
  auto x = MivViewParamsUpdateIntrinsics{};
  x.mvpui_num_view_updates_minus1(bitstream.getUint16());
  for (uint16_t i = 0; i <= x.mvpui_num_view_updates_minus1(); ++i) {
    x.mvpui_view_idx(i, bitstream.getUint16());
    x.camera_intrinsics(i) = CameraIntrinsics::decodeFrom(bitstream);
  }
  return x;
}

auto MivViewParamsUpdateIntrinsics::operator==(
    const MivViewParamsUpdateIntrinsics &other) const noexcept -> bool {
  return m_mvpui_num_view_updates_minus1 == other.m_mvpui_num_view_updates_minus1 &&
         m_mvpui_view_idx == other.m_mvpui_view_idx &&
         m_camera_intrinsics == other.m_camera_intrinsics;
}

auto MivViewParamsUpdateIntrinsics::operator!=(
    const MivViewParamsUpdateIntrinsics &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsUpdateDepthQuantization::mvpudq_num_view_updates_minus1() const noexcept
    -> uint16_t {
  return m_mvpudq_num_view_updates_minus1;
}

auto MivViewParamsUpdateDepthQuantization::mvpudq_view_idx(const uint16_t i) const -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpudq_view_idx.size());
  return m_mvpudq_view_idx[i];
}

auto MivViewParamsUpdateDepthQuantization::depth_quantization(const uint16_t i) const
    -> const DepthQuantization & {
  VERIFY_MIVBITSTREAM(i < m_depth_quantization.size());
  return m_depth_quantization[i];
}

auto MivViewParamsUpdateDepthQuantization::depth_quantization(const uint16_t i) noexcept
    -> DepthQuantization & {
  PRECONDITION(i < m_depth_quantization.size());
  return m_depth_quantization[i];
}

auto MivViewParamsUpdateDepthQuantization::mvpudq_num_view_updates_minus1(const uint16_t value)
    -> MivViewParamsUpdateDepthQuantization & {
  m_mvpudq_num_view_updates_minus1 = value;
  m_mvpudq_view_idx.resize(m_mvpudq_num_view_updates_minus1 + size_t{1});
  m_depth_quantization.resize(m_mvpudq_num_view_updates_minus1 + size_t{1});
  return *this;
}
auto MivViewParamsUpdateDepthQuantization::mvpudq_view_idx(const uint16_t i,
                                                           const uint16_t value) noexcept
    -> MivViewParamsUpdateDepthQuantization & {
  PRECONDITION(i < m_mvpudq_view_idx.size());
  m_mvpudq_view_idx[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const MivViewParamsUpdateDepthQuantization &x)
    -> std::ostream & {
  stream << "mvpudq_num_view_updates_minus1=" << x.mvpudq_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpudq_num_view_updates_minus1(); ++i) {
    stream << "mvpudq_view_idx[ " << i << " ]=" << x.mvpudq_view_idx(i) << '\n';
    x.depth_quantization(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateDepthQuantization::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpudq_num_view_updates_minus1());
  for (uint16_t i = 0; i <= mvpudq_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpudq_view_idx(i));
    depth_quantization(i).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateDepthQuantization::decodeFrom(Common::InputBitstream &bitstream)
    -> MivViewParamsUpdateDepthQuantization {
  auto x = MivViewParamsUpdateDepthQuantization{};
  x.mvpudq_num_view_updates_minus1(bitstream.getUint16());
  for (uint16_t i = 0; i <= x.mvpudq_num_view_updates_minus1(); ++i) {
    x.mvpudq_view_idx(i, bitstream.getUint16());
    x.depth_quantization(i) = DepthQuantization::decodeFrom(bitstream);
  }
  return x;
}

auto MivViewParamsUpdateDepthQuantization::operator==(
    const MivViewParamsUpdateDepthQuantization &other) const noexcept -> bool {
  return m_mvpudq_num_view_updates_minus1 == other.m_mvpudq_num_view_updates_minus1 &&
         m_mvpudq_view_idx == other.m_mvpudq_view_idx &&
         m_depth_quantization == other.m_depth_quantization;
}

auto MivViewParamsUpdateDepthQuantization::operator!=(
    const MivViewParamsUpdateDepthQuantization &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsUpdateChromaScaling::mvpucs_num_view_updates_minus1() const noexcept -> uint16_t {
  return m_mvpucs_num_view_updates_minus1;
}

auto MivViewParamsUpdateChromaScaling::mvpucs_view_idx(const uint16_t i) const -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpucs_view_idx.size());
  return m_mvpucs_view_idx[i];
}

auto MivViewParamsUpdateChromaScaling::chroma_scaling(const uint16_t i) const
    -> const ChromaScaling & {
  VERIFY_MIVBITSTREAM(i < m_chroma_scaling.size());
  return m_chroma_scaling[i];
}

auto MivViewParamsUpdateChromaScaling::chroma_scaling(const uint16_t i) noexcept
    -> ChromaScaling & {
  PRECONDITION(i < m_chroma_scaling.size());
  return m_chroma_scaling[i];
}

auto MivViewParamsUpdateChromaScaling::mvpucs_num_view_updates_minus1(const uint16_t value)
    -> MivViewParamsUpdateChromaScaling & {
  m_mvpucs_num_view_updates_minus1 = value;
  m_mvpucs_view_idx.resize(m_mvpucs_num_view_updates_minus1 + size_t{1});
  m_chroma_scaling.resize(m_mvpucs_num_view_updates_minus1 + size_t{1});
  return *this;
}
auto MivViewParamsUpdateChromaScaling::mvpucs_view_idx(const uint16_t i,
                                                       const uint16_t value) noexcept
    -> MivViewParamsUpdateChromaScaling & {
  PRECONDITION(i < m_mvpucs_view_idx.size());
  m_mvpucs_view_idx[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const MivViewParamsUpdateChromaScaling &x) -> std::ostream & {
  stream << "mvpucs_num_view_updates_minus1=" << x.mvpucs_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpucs_num_view_updates_minus1(); ++i) {
    stream << "mvpucs_view_idx[ " << i << " ]=" << x.mvpucs_view_idx(i) << '\n';
    x.chroma_scaling(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateChromaScaling::encodeTo(Common::OutputBitstream &bitstream,
                                                const MivViewParamsList &mvpl) const {
  bitstream.putUint16(mvpucs_num_view_updates_minus1());

  for (uint16_t i = 0; i <= mvpucs_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpucs_view_idx(i));
    chroma_scaling(i).encodeTo(bitstream, mvpl);
  }
}

auto MivViewParamsUpdateChromaScaling::decodeFrom(Common::InputBitstream &bitstream,
                                                  const MivViewParamsList &mvpl)
    -> MivViewParamsUpdateChromaScaling {
  auto x = MivViewParamsUpdateChromaScaling{};

  x.mvpucs_num_view_updates_minus1(bitstream.getUint16());

  for (uint16_t i = 0; i <= x.mvpucs_num_view_updates_minus1(); ++i) {
    x.mvpucs_view_idx(i, bitstream.getUint16());
    x.chroma_scaling(i) = ChromaScaling::decodeFrom(bitstream, mvpl);
  }
  return x;
}

auto MivViewParamsUpdateChromaScaling::operator==(
    const MivViewParamsUpdateChromaScaling &other) const noexcept -> bool {
  return m_mvpucs_num_view_updates_minus1 == other.m_mvpucs_num_view_updates_minus1 &&
         m_mvpucs_view_idx == other.m_mvpucs_view_idx && m_chroma_scaling == other.m_chroma_scaling;
}

auto MivViewParamsUpdateChromaScaling::operator!=(
    const MivViewParamsUpdateChromaScaling &other) const noexcept -> bool {
  return !operator==(other);
}

auto SensorExtrinsics::printTo(std::ostream &stream, uint16_t v, uint16_t s) const
    -> std::ostream & {
  fmt::print(stream, "se_sensor_pos_x[ {} ][ {} ]={}\n", v, s, se_sensor_pos_x());
  fmt::print(stream, "se_sensor_pos_y[ {} ][ {} ]={}\n", v, s, se_sensor_pos_y());
  fmt::print(stream, "se_sensor_pos_z[ {} ][ {} ]={}\n", v, s, se_sensor_pos_z());
  fmt::print(stream, "se_sensor_quat_x[ {} ][ {} ]={}\n", v, s, se_sensor_quat_x());
  fmt::print(stream, "se_sensor_quat_y[ {} ][ {} ]={}\n", v, s, se_sensor_quat_y());
  fmt::print(stream, "se_sensor_quat_z[ {} ][ {} ]={}\n", v, s, se_sensor_quat_z());
  return stream;
}

auto SensorExtrinsics::decodeFrom(Common::InputBitstream &bitstream) -> SensorExtrinsics {
  auto x = SensorExtrinsics{};
  x.se_sensor_pos_x(bitstream.getFloat32());
  x.se_sensor_pos_y(bitstream.getFloat32());
  x.se_sensor_pos_z(bitstream.getFloat32());
  x.se_sensor_quat_x(bitstream.getInt32());
  x.se_sensor_quat_y(bitstream.getInt32());
  x.se_sensor_quat_z(bitstream.getInt32());

  return x;
}

void SensorExtrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFloat32(se_sensor_pos_x());
  bitstream.putFloat32(se_sensor_pos_y());
  bitstream.putFloat32(se_sensor_pos_z());
  bitstream.putInt32(se_sensor_quat_x());
  bitstream.putInt32(se_sensor_quat_y());
  bitstream.putInt32(se_sensor_quat_z());
}

auto MivViewParamsUpdateSensorExtrinsics::mvpuse_num_updates_minus1() const noexcept -> uint16_t {
  return m_mvpuse_num_updates_minus1;
}

auto MivViewParamsUpdateSensorExtrinsics::mvpuse_view_idx(const uint16_t i) const -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpuse_view_idx.size());
  return m_mvpuse_view_idx[i];
}

auto MivViewParamsUpdateSensorExtrinsics::mvpuse_sensor_idx(const uint16_t i) const -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpuse_sensor_idx.size());
  return m_mvpuse_sensor_idx[i];
}

auto MivViewParamsUpdateSensorExtrinsics::sensor_extrinsics(const uint16_t v,
                                                            const uint16_t s) const
    -> const SensorExtrinsics & {
  VERIFY_MIVBITSTREAM(v < m_sensor_extrinsics.size());
  VERIFY_MIVBITSTREAM(s < m_sensor_extrinsics[v].size());
  return m_sensor_extrinsics[v][s];
}

auto MivViewParamsUpdateSensorExtrinsics::sensor_extrinsics(const uint16_t v,
                                                            const uint16_t s) noexcept
    -> SensorExtrinsics & {
  if (v >= m_sensor_extrinsics.size()) {
    m_sensor_extrinsics.resize(v + 1);
  }
  if (s >= m_sensor_extrinsics[v].size()) {
    m_sensor_extrinsics[v].resize(s + 1);
  }
  return m_sensor_extrinsics[v][s];
}

auto MivViewParamsUpdateSensorExtrinsics::mvpuse_num_updates_minus1(const uint16_t value)
    -> MivViewParamsUpdateSensorExtrinsics & {
  m_mvpuse_num_updates_minus1 = value;
  m_mvpuse_view_idx.resize(m_mvpuse_num_updates_minus1 + size_t{1});
  m_mvpuse_sensor_idx.resize(m_mvpuse_num_updates_minus1 + size_t{1});
  return *this;
}

auto MivViewParamsUpdateSensorExtrinsics::mvpuse_view_idx(const uint16_t i,
                                                          const uint16_t value) noexcept
    -> MivViewParamsUpdateSensorExtrinsics & {
  PRECONDITION(i < m_mvpuse_view_idx.size());
  m_mvpuse_view_idx[i] = value;
  return *this;
}

auto MivViewParamsUpdateSensorExtrinsics::mvpuse_sensor_idx(const uint16_t i,
                                                            const uint16_t value) noexcept
    -> MivViewParamsUpdateSensorExtrinsics & {
  PRECONDITION(i < m_mvpuse_sensor_idx.size());
  m_mvpuse_sensor_idx[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const MivViewParamsUpdateSensorExtrinsics &x)
    -> std::ostream & {
  stream << "mvpuse_num_updates_minus1=" << x.mvpuse_num_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpuse_num_updates_minus1(); ++i) {
    auto v = x.mvpuse_view_idx(i);
    auto s = x.mvpuse_sensor_idx(i);
    stream << "mvpuse_view_idx[ " << i << " ]=" << v << '\n';
    stream << "mvpuse_sensor_idx[ " << i << " ]=" << s << '\n';
    x.sensor_extrinsics(v, s).printTo(stream, v, s);
  }
  return stream;
}

void MivViewParamsUpdateSensorExtrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpuse_num_updates_minus1());
  for (uint16_t i = 0; i <= mvpuse_num_updates_minus1(); ++i) {
    auto v = mvpuse_view_idx(i);
    auto s = mvpuse_sensor_idx(i);
    bitstream.putUint16(v);
    bitstream.putUint16(s);
    sensor_extrinsics(v, s).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateSensorExtrinsics::decodeFrom(Common::InputBitstream &bitstream)
    -> MivViewParamsUpdateSensorExtrinsics {
  auto x = MivViewParamsUpdateSensorExtrinsics{};
  x.mvpuse_num_updates_minus1(bitstream.getUint16());
  for (uint16_t i = 0; i <= x.mvpuse_num_updates_minus1(); ++i) {
    auto v = bitstream.getUint16();
    auto s = bitstream.getUint16();
    x.mvpuse_view_idx(i, v);
    x.mvpuse_sensor_idx(i, s);
    x.sensor_extrinsics(v, s) = SensorExtrinsics::decodeFrom(bitstream);
  }
  return x;
}

auto MivViewParamsUpdateSensorExtrinsics::operator==(
    const MivViewParamsUpdateSensorExtrinsics &other) const noexcept -> bool {
  if (!(m_mvpuse_num_updates_minus1 == other.m_mvpuse_num_updates_minus1 &&
        m_mvpuse_view_idx == other.m_mvpuse_view_idx &&
        m_mvpuse_sensor_idx == other.m_mvpuse_sensor_idx)) {
    return false;
  }
  if (m_sensor_extrinsics.size() != other.m_sensor_extrinsics.size()) {
    return false;
  }
  for (uint16_t i = 0; i <= mvpuse_num_updates_minus1(); ++i) {
    auto v = mvpuse_view_idx(i);
    auto s = mvpuse_sensor_idx(i);
    if (sensor_extrinsics(v, s) != other.sensor_extrinsics(v, s)) {
      return false;
    }
  }

  return true;
}

auto MivViewParamsUpdateSensorExtrinsics::operator!=(
    const MivViewParamsUpdateSensorExtrinsics &other) const noexcept -> bool {
  return !operator==(other);
}

auto LightSourceExtrinsics::printTo(std::ostream &stream, uint16_t v, uint16_t s) const
    -> std::ostream & {
  fmt::print(stream, "lse_light_source_pos_x[ {} ][ {} ]={}\n", v, s, lse_light_source_pos_x());
  fmt::print(stream, "lse_light_source_pos_y[ {} ][ {} ]={}\n", v, s, lse_light_source_pos_y());
  fmt::print(stream, "lse_light_source_pos_z[ {} ][ {} ]={}\n", v, s, lse_light_source_pos_z());
  fmt::print(stream, "lse_light_source_quat_x[ {} ][ {} ]={}\n", v, s, lse_light_source_quat_x());
  fmt::print(stream, "lse_light_source_quat_y[ {} ][ {} ]={}\n", v, s, lse_light_source_quat_y());
  fmt::print(stream, "lse_light_source_quat_z[ {} ][ {} ]={}\n", v, s, lse_light_source_quat_z());
  return stream;
}

auto LightSourceExtrinsics::decodeFrom(Common::InputBitstream &bitstream) -> LightSourceExtrinsics {
  auto x = LightSourceExtrinsics{};

  x.lse_light_source_pos_x(bitstream.getFloat32());
  x.lse_light_source_pos_y(bitstream.getFloat32());
  x.lse_light_source_pos_z(bitstream.getFloat32());
  x.lse_light_source_quat_x(bitstream.getInt32());
  x.lse_light_source_quat_y(bitstream.getInt32());
  x.lse_light_source_quat_z(bitstream.getInt32());

  return x;
}

void LightSourceExtrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putFloat32(lse_light_source_pos_x());
  bitstream.putFloat32(lse_light_source_pos_y());
  bitstream.putFloat32(lse_light_source_pos_z());
  bitstream.putInt32(lse_light_source_quat_x());
  bitstream.putInt32(lse_light_source_quat_y());
  bitstream.putInt32(lse_light_source_quat_z());
}

auto MivViewParamsUpdateLightSourceExtrinsics::mvpulse_num_updates_minus1() const noexcept
    -> uint16_t {
  return m_mvpulse_num_updates_minus1;
}

auto MivViewParamsUpdateLightSourceExtrinsics::mvpulse_view_idx(const uint16_t i) const
    -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpulse_view_idx.size());
  return m_mvpulse_view_idx[i];
}

auto MivViewParamsUpdateLightSourceExtrinsics::mvpulse_sensor_idx(const uint16_t i) const
    -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpulse_sensor_idx.size());
  return m_mvpulse_sensor_idx[i];
}

auto MivViewParamsUpdateLightSourceExtrinsics::light_source_extrinsics(const uint16_t v,
                                                                       const uint16_t s) const
    -> const LightSourceExtrinsics & {
  VERIFY_MIVBITSTREAM(v < m_light_source_extrinsics.size());
  VERIFY_MIVBITSTREAM(s < m_light_source_extrinsics[v].size());
  return m_light_source_extrinsics[v][s];
}

auto MivViewParamsUpdateLightSourceExtrinsics::light_source_extrinsics(const uint16_t v,
                                                                       const uint16_t s) noexcept
    -> LightSourceExtrinsics & {
  if (v >= m_light_source_extrinsics.size()) {
    m_light_source_extrinsics.resize(v + 1);
  }
  if (s >= m_light_source_extrinsics[v].size()) {
    m_light_source_extrinsics[v].resize(s + 1);
  }
  return m_light_source_extrinsics[v][s];
}

auto MivViewParamsUpdateLightSourceExtrinsics::mvpulse_num_updates_minus1(const uint16_t value)
    -> MivViewParamsUpdateLightSourceExtrinsics & {
  m_mvpulse_num_updates_minus1 = value;
  m_mvpulse_view_idx.resize(m_mvpulse_num_updates_minus1 + size_t{1});
  m_mvpulse_sensor_idx.resize(m_mvpulse_num_updates_minus1 + size_t{1});
  return *this;
}
auto MivViewParamsUpdateLightSourceExtrinsics::mvpulse_view_idx(const uint16_t i,
                                                                const uint16_t value) noexcept
    -> MivViewParamsUpdateLightSourceExtrinsics & {
  PRECONDITION(i < m_mvpulse_view_idx.size());
  m_mvpulse_view_idx[i] = value;
  return *this;
}

auto MivViewParamsUpdateLightSourceExtrinsics::mvpulse_sensor_idx(const uint16_t i,
                                                                  const uint16_t value) noexcept
    -> MivViewParamsUpdateLightSourceExtrinsics & {
  PRECONDITION(i < m_mvpulse_sensor_idx.size());
  m_mvpulse_sensor_idx[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const MivViewParamsUpdateLightSourceExtrinsics &x)
    -> std::ostream & {
  stream << "mvpulse_num_updates_minus1=" << x.mvpulse_num_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpulse_num_updates_minus1(); ++i) {
    auto v = x.mvpulse_view_idx(i);
    auto s = x.mvpulse_sensor_idx(i);
    stream << "mvpulse_view_idx[ " << i << " ]=" << v << '\n';
    stream << "mvpulse_sensor_idx[ " << i << " ]=" << s << '\n';
    x.light_source_extrinsics(v, s).printTo(stream, v, s);
  }
  return stream;
}

void MivViewParamsUpdateLightSourceExtrinsics::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpulse_num_updates_minus1());
  for (uint16_t i = 0; i <= mvpulse_num_updates_minus1(); ++i) {
    auto v = mvpulse_view_idx(i);
    auto s = mvpulse_sensor_idx(i);
    bitstream.putUint16(v);
    bitstream.putUint16(s);
    light_source_extrinsics(v, s).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateLightSourceExtrinsics::decodeFrom(Common::InputBitstream &bitstream)
    -> MivViewParamsUpdateLightSourceExtrinsics {
  auto x = MivViewParamsUpdateLightSourceExtrinsics{};
  x.mvpulse_num_updates_minus1(bitstream.getUint16());
  for (uint16_t i = 0; i <= x.mvpulse_num_updates_minus1(); ++i) {
    auto v = bitstream.getUint16();
    auto s = bitstream.getUint16();
    x.mvpulse_view_idx(i, v);
    x.mvpulse_sensor_idx(i, s);
    x.light_source_extrinsics(v, s) = LightSourceExtrinsics::decodeFrom(bitstream);
  }
  return x;
}

auto MivViewParamsUpdateLightSourceExtrinsics::operator==(
    const MivViewParamsUpdateLightSourceExtrinsics &other) const noexcept -> bool {
  if (!(m_mvpulse_num_updates_minus1 == other.m_mvpulse_num_updates_minus1 &&
        m_mvpulse_view_idx == other.m_mvpulse_view_idx &&
        m_mvpulse_sensor_idx == other.m_mvpulse_sensor_idx)) {
    return false;
  }
  if (m_light_source_extrinsics.size() != other.m_light_source_extrinsics.size()) {
    return false;
  }
  for (uint16_t i = 0; i <= mvpulse_num_updates_minus1(); ++i) {
    auto v = mvpulse_view_idx(i);
    auto s = mvpulse_sensor_idx(i);
    if (light_source_extrinsics(v, s) != other.light_source_extrinsics(v, s)) {
      return false;
    }
  }

  return true;
}

auto MivViewParamsUpdateLightSourceExtrinsics::operator!=(
    const MivViewParamsUpdateLightSourceExtrinsics &other) const noexcept -> bool {
  return !operator==(other);
}

auto DistortionParameters::dp_coefficient(uint8_t i) const -> float {
  VERIFY(i < m_dp_coefficient.size());
  return m_dp_coefficient[i];
}
auto DistortionParameters::dp_model_id(uint8_t value) -> DistortionParameters & {
  VERIFY(value < 4);
  m_dp_model_id = value;
  m_dp_coefficient.resize(m_dp_model_id == 0   ? 0
                          : m_dp_model_id == 1 ? 4
                          : m_dp_model_id == 2 ? 5
                          : m_dp_model_id == 3 ? 8
                                               : 0);
  return *this;
}
auto DistortionParameters::dp_coefficient(uint8_t i, const float value) -> DistortionParameters & {
  VERIFY(i < m_dp_coefficient.size());
  m_dp_coefficient[i] = value;
  return *this;
}
auto DistortionParameters::printTo(std::ostream &stream, uint16_t v, uint16_t s) const
    -> std::ostream & {
  fmt::print(stream, "dp_model_id[ {} ][ {} ]={}\n", v, s, dp_model_id());
  fmt::print(stream, "dp_coefficient[ {} ][ {} ]={}\n", v, s, fmt::join(m_dp_coefficient, ", "));
  return stream;
}

auto DistortionParameters::decodeFrom(Common::InputBitstream &bitstream) -> DistortionParameters {
  auto x = DistortionParameters{};

  x.dp_model_id(bitstream.getUExpGolomb<uint8_t>());
  VERIFY_MIVBITSTREAM(x.dp_model_id() < 4);

  for (uint8_t i = 0; i < static_cast<uint8_t>(x.m_dp_coefficient.size()); i++) {
    x.dp_coefficient(i, bitstream.getFloat32());
  }

  return x;
}

void DistortionParameters::encodeTo(Common::OutputBitstream &bitstream) const {
  VERIFY_MIVBITSTREAM(dp_model_id() < 4);
  bitstream.putUExpGolomb<uint8_t>(dp_model_id());
  for (uint8_t i = 0; i < static_cast<uint8_t>(m_dp_coefficient.size()); i++) {
    bitstream.putFloat32(dp_coefficient(i));
  }
}

auto MivViewParamsUpdateDistortionParameters::mvpudp_num_updates_minus1() const noexcept
    -> uint16_t {
  return m_mvpudp_num_updates_minus1;
}

auto MivViewParamsUpdateDistortionParameters::mvpudp_view_idx(const uint16_t i) const -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpudp_view_idx.size());
  return m_mvpudp_view_idx[i];
}

auto MivViewParamsUpdateDistortionParameters::mvpudp_sensor_idx(const uint16_t i) const
    -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpudp_sensor_idx.size());
  return m_mvpudp_sensor_idx[i];
}

auto MivViewParamsUpdateDistortionParameters::distortion_parameters(const uint16_t v,
                                                                    const uint16_t s) const
    -> const DistortionParameters & {
  VERIFY_MIVBITSTREAM(v < m_distortion_parameters.size());
  VERIFY_MIVBITSTREAM(s < m_distortion_parameters[v].size());
  return m_distortion_parameters[v][s];
}

auto MivViewParamsUpdateDistortionParameters::distortion_parameters(const uint16_t v,
                                                                    const uint16_t s) noexcept
    -> DistortionParameters & {
  if (v >= m_distortion_parameters.size()) {
    m_distortion_parameters.resize(v + 1);
  }
  if (s >= m_distortion_parameters[v].size()) {
    m_distortion_parameters[v].resize(s + 1);
  }
  return m_distortion_parameters[v][s];
}

auto MivViewParamsUpdateDistortionParameters::mvpudp_num_updates_minus1(const uint16_t value)
    -> MivViewParamsUpdateDistortionParameters & {
  m_mvpudp_num_updates_minus1 = value;
  m_mvpudp_view_idx.resize(m_mvpudp_num_updates_minus1 + size_t{1});
  m_mvpudp_sensor_idx.resize(m_mvpudp_num_updates_minus1 + size_t{1});
  return *this;
}
auto MivViewParamsUpdateDistortionParameters::mvpudp_view_idx(const uint16_t i,
                                                              const uint16_t value) noexcept
    -> MivViewParamsUpdateDistortionParameters & {
  PRECONDITION(i < m_mvpudp_view_idx.size());
  m_mvpudp_view_idx[i] = value;
  return *this;
}

auto MivViewParamsUpdateDistortionParameters::mvpudp_sensor_idx(const uint16_t i,
                                                                const uint16_t value) noexcept
    -> MivViewParamsUpdateDistortionParameters & {
  PRECONDITION(i < m_mvpudp_sensor_idx.size());
  m_mvpudp_sensor_idx[i] = value;
  return *this;
}

auto operator<<(std::ostream &stream, const MivViewParamsUpdateDistortionParameters &x)
    -> std::ostream & {
  fmt::print(stream, "mvpudp_num_updates_minus1={}\n", x.mvpudp_num_updates_minus1());
  for (uint16_t i = 0; i <= x.mvpudp_num_updates_minus1(); ++i) {
    auto v = x.mvpudp_view_idx(i);
    auto s = x.mvpudp_sensor_idx(i);
    fmt::print(stream, "mvpudp_view_idx[ {} ]={}\n", i, v);
    fmt::print(stream, "mvpudp_sensor_idx[ {} ]={}\n", i, s);
    x.distortion_parameters(v, s).printTo(stream, v, s);
  }
  return stream;
}

void MivViewParamsUpdateDistortionParameters::encodeTo(Common::OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpudp_num_updates_minus1());
  for (uint16_t i = 0; i <= mvpudp_num_updates_minus1(); ++i) {
    auto v = mvpudp_view_idx(i);
    auto s = mvpudp_sensor_idx(i);
    bitstream.putUint16(v);
    bitstream.putUint16(s);
    distortion_parameters(v, s).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateDistortionParameters::decodeFrom(Common::InputBitstream &bitstream)
    -> MivViewParamsUpdateDistortionParameters {
  auto x = MivViewParamsUpdateDistortionParameters{};
  x.mvpudp_num_updates_minus1(bitstream.getUint16());
  for (uint16_t i = 0; i <= x.mvpudp_num_updates_minus1(); ++i) {
    auto v = bitstream.getUint16();
    auto s = bitstream.getUint16();
    x.mvpudp_view_idx(i, v);
    x.mvpudp_sensor_idx(i, s);
    x.distortion_parameters(v, s) = DistortionParameters::decodeFrom(bitstream);
  }
  return x;
}

auto MivViewParamsUpdateDistortionParameters::operator==(
    const MivViewParamsUpdateDistortionParameters &other) const noexcept -> bool {
  if (!(m_mvpudp_num_updates_minus1 == other.m_mvpudp_num_updates_minus1 &&
        m_mvpudp_view_idx == other.m_mvpudp_view_idx &&
        m_mvpudp_sensor_idx == other.m_mvpudp_sensor_idx)) {
    return false;
  }
  if (m_distortion_parameters.size() != other.m_distortion_parameters.size()) {
    return false;
  }
  for (uint16_t i = 0; i <= mvpudp_num_updates_minus1(); ++i) {
    auto v = mvpudp_view_idx(i);
    auto s = mvpudp_sensor_idx(i);
    if (distortion_parameters(v, s) != other.distortion_parameters(v, s)) {
      return false;
    }
  }

  return true;
}

auto MivViewParamsUpdateDistortionParameters::operator!=(
    const MivViewParamsUpdateDistortionParameters &other) const noexcept -> bool {
  return !operator==(other);
}

} // namespace TMIV::MivBitstream
