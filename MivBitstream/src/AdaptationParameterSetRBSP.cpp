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

#include <TMIV/MivBitstream/AdaptationParameterSetRBSP.h>

#include "verify.h"

#include <cmath>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, const MvpUpdateMode x) -> ostream & {
  switch (x) {
  case MvpUpdateMode::VPL_INITLIST:
    return stream << "VPL_INITLIST";
  case MvpUpdateMode::VPL_UPD_EXT:
    return stream << "VPL_UPD_EXT";
  case MvpUpdateMode::VPL_UPD_INT:
    return stream << "VPL_UPD_INT";
  case MvpUpdateMode::VPL_EXT_INT:
    return stream << "VPL_EXT_INT";
  default:
    MIVBITSTREAM_ERROR("Unknown update mode");
  }
}
auto operator<<(ostream &stream, const CiCamType x) -> ostream & {
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

auto CameraIntrinsics::projectionPlaneSize() const -> Vec2i {
  return {ci_projection_plane_width_minus1() + 1, ci_projection_plane_height_minus1() + 1};
}

auto CameraIntrinsics::ci_erp_phi_min() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_phi_min.has_value());
  return *m_ci_erp_phi_min;
}

auto CameraIntrinsics::ci_erp_phi_max() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_phi_max.has_value());
  return *m_ci_erp_phi_max;
}

auto CameraIntrinsics::ci_erp_theta_min() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_theta_min.has_value());
  return *m_ci_erp_theta_min;
}

auto CameraIntrinsics::ci_erp_theta_max() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::equirectangular);
  VERIFY_MIVBITSTREAM(m_ci_erp_theta_max.has_value());
  return *m_ci_erp_theta_max;
}

auto CameraIntrinsics::ci_perspective_focal_hor() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_focal_hor.has_value());
  return *m_ci_perspective_focal_hor;
}

auto CameraIntrinsics::ci_perspective_focal_ver() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_focal_ver.has_value());
  return *m_ci_perspective_focal_ver;
}

auto CameraIntrinsics::ci_perspective_center_hor() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_center_hor.has_value());
  return *m_ci_perspective_center_hor;
}

auto CameraIntrinsics::ci_perspective_center_ver() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::perspective);
  VERIFY_MIVBITSTREAM(m_ci_perspective_center_ver.has_value());
  return *m_ci_perspective_center_ver;
}

auto CameraIntrinsics::ci_ortho_width() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::orthographic);
  VERIFY_MIVBITSTREAM(m_ci_ortho_width.has_value());
  return *m_ci_ortho_width;
}

auto CameraIntrinsics::ci_ortho_height() const noexcept -> float {
  VERIFY_MIVBITSTREAM(ci_cam_type() == CiCamType::orthographic);
  VERIFY_MIVBITSTREAM(m_ci_ortho_height.has_value());
  return *m_ci_ortho_height;
}

auto CameraIntrinsics::printTo(ostream &stream, uint16_t viewId) const -> ostream & {
  stream << "ci_cam_type[ " << viewId << " ]=" << ci_cam_type() << '\n';
  stream << "ci_projection_plane_width_minus1[ " << viewId
         << " ]=" << ci_projection_plane_width_minus1() << '\n';
  stream << "ci_projection_plane_height_minus1[ " << viewId
         << " ]=" << ci_projection_plane_height_minus1() << '\n';

  switch (ci_cam_type()) {
  case CiCamType::equirectangular:
    stream << "ci_erp_phi_min[ " << viewId << " ]=" << ci_erp_phi_min() << '\n';
    stream << "ci_erp_phi_max[ " << viewId << " ]=" << ci_erp_phi_max() << '\n';
    stream << "ci_erp_theta_min[ " << viewId << " ]=" << ci_erp_theta_min() << '\n';
    stream << "ci_erp_theta_max[ " << viewId << " ]=" << ci_erp_theta_max() << '\n';
    return stream;

  case CiCamType::perspective:
    stream << "ci_perspective_focal_hor[ " << viewId << " ]=" << ci_perspective_focal_hor() << '\n';
    stream << "ci_perspective_focal_ver[ " << viewId << " ]=" << ci_perspective_focal_ver() << '\n';
    stream << "ci_perspective_center_hor[ " << viewId << " ]=" << ci_perspective_center_hor()
           << '\n';
    stream << "ci_perspective_center_ver[ " << viewId << " ]=" << ci_perspective_center_ver()
           << '\n';
    return stream;

  case CiCamType::orthographic:
    stream << "ci_ortho_width[ " << viewId << " ]=" << ci_ortho_width() << '\n';
    stream << "ci_ortho_height[ " << viewId << " ]=" << ci_ortho_height() << '\n';
    return stream;

  default:
    MIVBITSTREAM_ERROR("Unknown cam type");
  }
}

auto CameraIntrinsics::decodeFrom(InputBitstream &bitstream) -> CameraIntrinsics {
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

void CameraIntrinsics::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint8(uint8_t(ci_cam_type()));
  bitstream.putUint16(ci_projection_plane_width_minus1());
  bitstream.putUint16(ci_projection_plane_height_minus1());

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
    MIVBITSTREAM_ERROR("Unknown cam type");
  }
}

auto CameraExtrinsics::position() const noexcept -> Vec3f {
  return {ce_view_pos_x(), ce_view_pos_y(), ce_view_pos_z()};
}

auto CameraExtrinsics::rotation() const noexcept -> QuatF {
  const auto x = ce_view_quat_x();
  const auto y = ce_view_quat_y();
  const auto z = ce_view_quat_z();
  const auto w = sqrt(max(0.F, 1.F - x * x - y * y - z * z));

  return {x, y, z, w};
}

auto CameraExtrinsics::position(Vec3f r) noexcept -> CameraExtrinsics & {
  ce_view_pos_x(r.x());
  ce_view_pos_y(r.y());
  ce_view_pos_z(r.z());
  return *this;
}

auto CameraExtrinsics::rotation(QuatF q) noexcept -> CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(normalized(q));
  ce_view_quat_x(q.x());
  ce_view_quat_y(q.y());
  ce_view_quat_z(q.z());
  return *this;
}

auto CameraExtrinsics::printTo(ostream &stream, uint16_t viewId) const -> ostream & {
  stream << "ce_view_pos_x[ " << viewId << " ]=" << ce_view_pos_x() << '\n';
  stream << "ce_view_pos_y[ " << viewId << " ]=" << ce_view_pos_y() << '\n';
  stream << "ce_view_pos_z[ " << viewId << " ]=" << ce_view_pos_z() << '\n';
  stream << "ce_view_quat_x[ " << viewId << " ]=" << ce_view_quat_x() << '\n';
  stream << "ce_view_quat_y[ " << viewId << " ]=" << ce_view_quat_y() << '\n';
  stream << "ce_view_quat_z[ " << viewId << " ]=" << ce_view_quat_z() << '\n';
  return stream;
}

auto CameraExtrinsics::decodeFrom(InputBitstream &bitstream) -> CameraExtrinsics {
  auto x = CameraExtrinsics{};

  x.ce_view_pos_x(bitstream.getFloat32());
  x.ce_view_pos_y(bitstream.getFloat32());
  x.ce_view_pos_z(bitstream.getFloat32());
  x.ce_view_quat_x(bitstream.getFloat32());
  x.ce_view_quat_y(bitstream.getFloat32());
  x.ce_view_quat_z(bitstream.getFloat32());

  return x;
}

void CameraExtrinsics::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putFloat32(ce_view_pos_x());
  bitstream.putFloat32(ce_view_pos_y());
  bitstream.putFloat32(ce_view_pos_z());
  bitstream.putFloat32(ce_view_quat_x());
  bitstream.putFloat32(ce_view_quat_y());
  bitstream.putFloat32(ce_view_quat_z());
}

auto DepthQuantization::printTo(ostream &stream, uint16_t viewId) const -> ostream & {
  VERIFY_MIVBITSTREAM(dq_quantization_law() == 0);
  stream << "dq_quantization_law[ " << viewId << " ]=" << int(dq_quantization_law())
         << "\ndq_norm_disp_low[ " << viewId << " ]=" << dq_norm_disp_low()
         << "\ndq_norm_disp_high[ " << viewId << " ]=" << dq_norm_disp_high()
         << "\ndq_depth_occ_map_threshold_default[ " << viewId
         << " ]=" << dq_depth_occ_map_threshold_default() << '\n';
  return stream;
}

auto DepthQuantization::decodeFrom(InputBitstream &bitstream) -> DepthQuantization {
  auto x = DepthQuantization{};

  const auto dq_quantization_law = bitstream.getUint8();
  VERIFY_MIVBITSTREAM(dq_quantization_law == 0);

  x.dq_norm_disp_low(bitstream.getFloat32());
  x.dq_norm_disp_high(bitstream.getFloat32());

  x.dq_depth_occ_map_threshold_default(bitstream.getUExpGolomb<uint32_t>());

  return x;
}

void DepthQuantization::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint8(dq_quantization_law());
  bitstream.putFloat32(dq_norm_disp_low());
  bitstream.putFloat32(dq_norm_disp_high());

  bitstream.putUExpGolomb(dq_depth_occ_map_threshold_default());
}

PruningChildren::PruningChildren(vector<uint16_t> pc_child_id) : m_pc_child_id{move(pc_child_id)} {}

auto PruningChildren::pc_is_leaf_flag() const noexcept -> bool { return m_pc_child_id.empty(); }

auto PruningChildren::pc_num_children_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(!pc_is_leaf_flag());
  return uint16_t(m_pc_child_id.size() - 1);
}

auto PruningChildren::pc_child_id(uint16_t i) const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_pc_child_id.size());
  return m_pc_child_id[i];
}

auto PruningChildren::pc_child_id(std::uint16_t i, std::uint16_t value) noexcept
    -> PruningChildren & {
  VERIFY_MIVBITSTREAM(i < m_pc_child_id.size());
  m_pc_child_id[i] = value;
  return *this;
}

auto PruningChildren::printTo(ostream &stream, uint16_t viewId) const -> ostream & {
  stream << "pc_is_leaf_flag[ " << viewId << " ]=" << boolalpha << pc_is_leaf_flag() << '\n';
  if (!pc_is_leaf_flag()) {
    stream << "pc_num_children_minus1[ " << viewId << " ]=" << pc_num_children_minus1() << '\n';
    for (auto i = 0; i <= pc_num_children_minus1(); ++i) {
      stream << "pc_child_id[ " << viewId << " ][ " << i << " ]=" << pc_child_id(i) << '\n';
    }
  }
  return stream;
}

auto PruningChildren::operator==(const PruningChildren &other) const noexcept -> bool {
  return m_pc_child_id == other.m_pc_child_id;
}

auto PruningChildren::operator!=(const PruningChildren &other) const noexcept -> bool {
  return !operator==(other);
}

auto PruningChildren::decodeFrom(InputBitstream &bitstream, uint16_t mvp_num_views_minus1)
    -> PruningChildren {
  const auto pc_is_leaf_flag = bitstream.getFlag();
  if (pc_is_leaf_flag) {
    return {};
  }

  const auto pc_num_children_minus1 = bitstream.getUVar<size_t>(mvp_num_views_minus1);
  auto x = vector<uint16_t>(pc_num_children_minus1 + 1);

  for (uint16_t &i : x) {
    i = bitstream.getUVar<uint16_t>(mvp_num_views_minus1 + 1);
  }

  return PruningChildren{x};
}

void PruningChildren::encodeTo(OutputBitstream &bitstream, uint16_t mvp_num_views_minus1) const {
  bitstream.putFlag(pc_is_leaf_flag());
  if (!pc_is_leaf_flag()) {
    bitstream.putUVar(pc_num_children_minus1(), mvp_num_views_minus1);
    for (uint16_t i = 0; i <= pc_num_children_minus1(); ++i) {
      bitstream.putUVar(pc_child_id(i), mvp_num_views_minus1 + 1);
    }
  }
}

auto MivViewParamsList::mvp_num_views_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(!m_camera_extrinsics.empty());
  return uint16_t(m_camera_extrinsics.size() - 1);
}

auto MivViewParamsList::camera_extrinsics(const uint16_t viewId) const noexcept
    -> const CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_extrinsics.size());
  return m_camera_extrinsics[viewId];
}

auto MivViewParamsList::camera_intrinsics(uint16_t viewId) const noexcept
    -> const CameraIntrinsics & {
  if (mvp_intrinsic_params_equal_flag()) {
    viewId = 0; // Convenience
  }
  VERIFY_MIVBITSTREAM(viewId < m_camera_intrinsics.size());
  return m_camera_intrinsics[viewId];
}

auto MivViewParamsList::depth_quantization(uint16_t viewId) const noexcept
    -> const DepthQuantization & {
  if (mvp_depth_quantization_params_equal_flag()) {
    viewId = 0; // Convenience
  }
  VERIFY_MIVBITSTREAM(viewId < m_depth_quantization.size());
  return m_depth_quantization[viewId];
}

auto MivViewParamsList::pruning_children(const uint16_t viewId) const noexcept
    -> const PruningChildren & {
  VERIFY_MIVBITSTREAM(mvp_pruning_graph_params_present_flag());
  VERIFY_MIVBITSTREAM(viewId < m_pruning_children.size());
  return m_pruning_children[viewId];
}

auto MivViewParamsList::mvp_num_views_minus1(const uint16_t value) noexcept -> MivViewParamsList & {
  m_camera_extrinsics.resize(value + 1);
  return *this;
}

auto MivViewParamsList::mvp_intrinsic_params_equal_flag(const bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_intrinsic_params_equal_flag = value;
  m_camera_intrinsics.resize(value ? 1U : m_camera_extrinsics.size());
  return *this;
}

auto MivViewParamsList::mvp_depth_quantization_params_equal_flag(const bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_depth_quantization_params_equal_flag = value;
  m_depth_quantization.resize(value ? 1U : m_camera_extrinsics.size());
  return *this;
}

auto MivViewParamsList::mvp_pruning_graph_params_present_flag(const bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_pruning_graph_params_present_flag = value;
  m_pruning_children.resize(value ? m_camera_extrinsics.size() : 0U);
  return *this;
}

auto MivViewParamsList::camera_extrinsics(const uint16_t viewId) noexcept -> CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_extrinsics.size());
  return m_camera_extrinsics[viewId];
}

auto MivViewParamsList::camera_intrinsics(const uint16_t viewId) noexcept -> CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_intrinsics.size());
  return m_camera_intrinsics[viewId];
}

auto MivViewParamsList::depth_quantization(const uint16_t viewId) noexcept -> DepthQuantization & {
  VERIFY_MIVBITSTREAM(viewId < m_depth_quantization.size());
  return m_depth_quantization[viewId];
}

auto MivViewParamsList::pruning_children(const uint16_t viewId) noexcept -> PruningChildren & {
  VERIFY_MIVBITSTREAM(viewId < m_pruning_children.size());
  return m_pruning_children[viewId];
}

auto operator<<(ostream &stream, const MivViewParamsList &x) -> ostream & {
  stream << "mvp_num_views_minus1=" << x.mvp_num_views_minus1() << '\n';
  for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
    x.camera_extrinsics(v).printTo(stream, v);
  }

  stream << "mvp_intrinsic_params_equal_flag=" << boolalpha << x.mvp_intrinsic_params_equal_flag()
         << '\n';
  if (x.mvp_intrinsic_params_equal_flag()) {
    x.camera_intrinsics(0).printTo(stream, 0);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.camera_intrinsics(v).printTo(stream, v);
    }
  }

  stream << "mvp_depth_quantization_params_equal_flag=" << boolalpha
         << x.mvp_depth_quantization_params_equal_flag() << '\n';
  if (x.mvp_depth_quantization_params_equal_flag()) {
    x.depth_quantization(0).printTo(stream, 0);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.depth_quantization(v).printTo(stream, v);
    }
  }

  stream << "mvp_pruning_graph_params_present_flag=" << boolalpha
         << x.mvp_pruning_graph_params_present_flag() << '\n';
  if (x.mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.pruning_children(v).printTo(stream, v);
    }
  }
  return stream;
}

auto MivViewParamsList::operator==(const MivViewParamsList &other) const noexcept -> bool {
  return m_camera_extrinsics == other.m_camera_extrinsics &&
         m_mvp_intrinsic_params_equal_flag == other.m_mvp_intrinsic_params_equal_flag &&
         m_camera_intrinsics == other.m_camera_intrinsics &&
         m_mvp_depth_quantization_params_equal_flag ==
             other.m_mvp_depth_quantization_params_equal_flag &&
         m_depth_quantization == other.m_depth_quantization &&
         m_mvp_pruning_graph_params_present_flag == other.m_mvp_pruning_graph_params_present_flag &&
         m_pruning_children == other.m_pruning_children;
}

auto MivViewParamsList::operator!=(const MivViewParamsList &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsList::decodeFrom(InputBitstream &bitstream) -> MivViewParamsList {
  auto x = MivViewParamsList{};

  x.mvp_num_views_minus1(bitstream.getUint16());

  for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
    x.camera_extrinsics(v) = CameraExtrinsics::decodeFrom(bitstream);
  }

  x.mvp_intrinsic_params_equal_flag(bitstream.getFlag());

  if (x.mvp_intrinsic_params_equal_flag()) {
    x.camera_intrinsics() = CameraIntrinsics::decodeFrom(bitstream);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.camera_intrinsics(v) = CameraIntrinsics::decodeFrom(bitstream);
    }
  }

  x.mvp_depth_quantization_params_equal_flag(bitstream.getFlag());

  if (x.mvp_depth_quantization_params_equal_flag()) {
    x.depth_quantization() = DepthQuantization::decodeFrom(bitstream);
  } else {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.depth_quantization(v) = DepthQuantization::decodeFrom(bitstream);
    }
  }

  x.mvp_pruning_graph_params_present_flag(bitstream.getFlag());

  if (x.mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.pruning_children(v) = PruningChildren::decodeFrom(bitstream, x.mvp_num_views_minus1());
    }
  }
  return x;
}

void MivViewParamsList::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint16(mvp_num_views_minus1());

  for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
    camera_extrinsics(v).encodeTo(bitstream);
  }

  bitstream.putFlag(mvp_intrinsic_params_equal_flag());

  if (mvp_intrinsic_params_equal_flag()) {
    camera_intrinsics().encodeTo(bitstream);
  } else {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      camera_intrinsics(v).encodeTo(bitstream);
    }
  }

  bitstream.putFlag(mvp_depth_quantization_params_equal_flag());

  if (mvp_depth_quantization_params_equal_flag()) {
    depth_quantization().encodeTo(bitstream);
  } else {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      depth_quantization(v).encodeTo(bitstream);
    }
  }

  bitstream.putFlag(mvp_pruning_graph_params_present_flag());

  if (mvp_pruning_graph_params_present_flag()) {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      pruning_children(v).encodeTo(bitstream, mvp_num_views_minus1());
    }
  }
}

auto AdaptationParameterSetRBSP::aps_miv_view_params_list_update_mode(
    const MvpUpdateMode value) noexcept -> AdaptationParameterSetRBSP & {
  m_aps_miv_view_params_list_update_mode = value;
  return *this;
}

auto AdaptationParameterSetRBSP::aps_miv_view_params_list_update_mode() const noexcept
    -> MvpUpdateMode {
  VERIFY_MIVBITSTREAM(m_aps_miv_view_params_list_update_mode.has_value());
  return *m_aps_miv_view_params_list_update_mode;
}

auto AdaptationParameterSetRBSP::miv_view_params_list() const noexcept
    -> const MivViewParamsList & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_list.has_value());
  return *m_miv_view_params_list;
}

auto AdaptationParameterSetRBSP::miv_view_params_update_extrinsics() const noexcept
    -> const MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_extrinsics.has_value());
  return *m_miv_view_params_update_extrinsics;
}

auto AdaptationParameterSetRBSP::miv_view_params_update_intrinsics() const noexcept
    -> const MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_intrinsics.has_value());
  return *m_miv_view_params_update_intrinsics;
}

auto operator<<(ostream &stream, const AdaptationParameterSetRBSP &x) -> ostream & {
  stream << "aps_adaptation_parameter_set_id=" << int(x.aps_adaptation_parameter_set_id())
         << "\naps_camera_params_present_flag=" << boolalpha << x.aps_camera_params_present_flag()
         << "\naps_miv_view_params_list_present_flag=" << boolalpha
         << x.aps_miv_view_params_list_present_flag() << '\n';

  if (x.aps_miv_view_params_list_present_flag()) {
    stream << "aps_miv_view_params_list_update_mode=" << x.aps_miv_view_params_list_update_mode()
           << '\n';
    switch (x.aps_miv_view_params_list_update_mode()) {
    case MvpUpdateMode::VPL_INITLIST:
      stream << x.miv_view_params_list();
      break;
    case MvpUpdateMode::VPL_UPD_EXT:
      stream << x.miv_view_params_update_extrinsics();
      break;
    case MvpUpdateMode::VPL_UPD_INT:
      stream << x.miv_view_params_update_intrinsics();
      break;
    case MvpUpdateMode::VPL_EXT_INT:
      stream << x.miv_view_params_update_extrinsics();
      stream << x.miv_view_params_update_intrinsics();
      break;
    default:
      MIVBITSTREAM_ERROR("Unknown update mode");
    }
  }
  stream << "aps_extension2_flag=" << boolalpha << x.aps_extension2_flag() << '\n';
  return stream;
}

auto AdaptationParameterSetRBSP::operator==(const AdaptationParameterSetRBSP &other) const noexcept
    -> bool {
  if (aps_adaptation_parameter_set_id() != other.aps_adaptation_parameter_set_id() ||
      aps_miv_view_params_list_present_flag() != other.aps_miv_view_params_list_present_flag()) {
    return false;
  }
  if (!aps_miv_view_params_list_present_flag()) {
    return true;
  }
  switch (aps_miv_view_params_list_update_mode()) {
  case MvpUpdateMode::VPL_INITLIST:
    return miv_view_params_list() == other.miv_view_params_list();
  case MvpUpdateMode::VPL_UPD_EXT:
    return miv_view_params_update_extrinsics() == other.miv_view_params_update_extrinsics();
  case MvpUpdateMode::VPL_UPD_INT:
    return miv_view_params_update_intrinsics() == other.miv_view_params_update_intrinsics();
  case MvpUpdateMode::VPL_EXT_INT:
    return miv_view_params_update_extrinsics() == other.miv_view_params_update_extrinsics() &&
           miv_view_params_update_intrinsics() == other.miv_view_params_update_intrinsics();
  default:
    MIVBITSTREAM_ERROR("Unknown update mode");
  }
}

auto AdaptationParameterSetRBSP::operator!=(const AdaptationParameterSetRBSP &other) const noexcept
    -> bool {
  return !operator==(other);
}

auto AdaptationParameterSetRBSP::decodeFrom(istream &stream) -> AdaptationParameterSetRBSP {
  InputBitstream bitstream{stream};

  auto x = AdaptationParameterSetRBSP{};

  x.aps_adaptation_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());

  const auto aps_camera_params_present_flag = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(!aps_camera_params_present_flag);

  const auto aps_extension_bit_equal_to_one = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(aps_extension_bit_equal_to_one);

  x.aps_miv_view_params_list_present_flag(bitstream.getFlag());

  if (x.aps_miv_view_params_list_present_flag()) {
    x.aps_miv_view_params_list_update_mode(bitstream.readBits<MvpUpdateMode>(2));

    switch (x.aps_miv_view_params_list_update_mode()) {
    case MvpUpdateMode::VPL_INITLIST:
      x.miv_view_params_list() = MivViewParamsList::decodeFrom(bitstream);
      break;
    case MvpUpdateMode::VPL_UPD_EXT:
      x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
      break;
    case MvpUpdateMode::VPL_UPD_INT:
      x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
      break;
    case MvpUpdateMode::VPL_EXT_INT:
      x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
      x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
      break;
    }
  }

  const auto aps_extension2_flag = bitstream.getFlag();
  VERIFY_MIVBITSTREAM(!aps_extension2_flag);
  bitstream.rbspTrailingBits();

  return x;
}

void AdaptationParameterSetRBSP::encodeTo(ostream &stream) const {
  OutputBitstream bitstream{stream};

  bitstream.putUExpGolomb(aps_adaptation_parameter_set_id());

  VERIFY_MIVBITSTREAM(!aps_camera_params_present_flag());
  bitstream.putFlag(aps_camera_params_present_flag());

  constexpr auto aps_extension_bit_equal_to_one = true;
  bitstream.putFlag(aps_extension_bit_equal_to_one);

  bitstream.putFlag(aps_miv_view_params_list_present_flag());

  if (aps_miv_view_params_list_present_flag()) {
    bitstream.writeBits(aps_miv_view_params_list_update_mode(), 2);
    switch (aps_miv_view_params_list_update_mode()) {
    case MvpUpdateMode::VPL_INITLIST:
      miv_view_params_list().encodeTo(bitstream);
      break;
    case MvpUpdateMode::VPL_UPD_EXT:
      miv_view_params_update_extrinsics().encodeTo(bitstream);
      break;
    case MvpUpdateMode::VPL_UPD_INT:
      miv_view_params_update_intrinsics().encodeTo(bitstream);
      break;
    case MvpUpdateMode::VPL_EXT_INT:
      miv_view_params_update_extrinsics().encodeTo(bitstream);
      miv_view_params_update_intrinsics().encodeTo(bitstream);
      break;
    }
  }

  VERIFY_MIVBITSTREAM(!aps_extension2_flag());
  bitstream.putFlag(aps_extension2_flag());
  bitstream.rbspTrailingBits();
}

auto MivViewParamsUpdateExtrinsics::mvpue_num_view_updates_minus1() const noexcept
    -> std::uint16_t {
  return m_mvpue_num_view_updates_minus1;
}

auto MivViewParamsUpdateExtrinsics::mvpue_view_idx(const uint16_t i) const noexcept
    -> std::uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpue_view_idx.size());
  return m_mvpue_view_idx.at(i);
}

auto MivViewParamsUpdateExtrinsics::camera_extrinsics(const uint16_t i) const noexcept
    -> const CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_extrinsics.size());
  return m_camera_extrinsics.at(i);
}
auto MivViewParamsUpdateExtrinsics::camera_extrinsics(const uint16_t i) noexcept
    -> CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_extrinsics.size());
  return m_camera_extrinsics.at(i);
}

auto MivViewParamsUpdateExtrinsics::mvpue_num_view_updates_minus1(const uint16_t value) noexcept
    -> MivViewParamsUpdateExtrinsics & {
  m_mvpue_num_view_updates_minus1 = value;
  m_mvpue_view_idx.resize(value + 1);
  m_camera_extrinsics.resize(value + 1);
  return *this;
}
auto MivViewParamsUpdateExtrinsics::mvpue_view_idx(const std::uint16_t i,
                                                   const uint16_t value) noexcept
    -> MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_extrinsics.size());
  m_mvpue_view_idx.at(i) = value;
  return *this;
}

auto operator<<(ostream &stream, const MivViewParamsUpdateExtrinsics &x) -> ostream & {
  stream << "mvpue_num_view_updates_minus1=" << x.mvpue_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpue_num_view_updates_minus1(); ++i) {
    stream << "mvpue_view_idx[ " << i << " ]=" << x.mvpue_view_idx(i) << '\n';
    x.camera_extrinsics(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateExtrinsics::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpue_num_view_updates_minus1());
  for (uint16_t i = 0; i <= mvpue_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpue_view_idx(i));
    camera_extrinsics(i).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateExtrinsics::decodeFrom(InputBitstream &bitstream)
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
         std::equal(m_mvpue_view_idx.begin(), m_mvpue_view_idx.end(),
                    other.m_mvpue_view_idx.begin()) &&
         m_camera_extrinsics == other.m_camera_extrinsics;
}

auto MivViewParamsUpdateExtrinsics::operator!=(
    const MivViewParamsUpdateExtrinsics &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsUpdateIntrinsics::mvpue_num_view_updates_minus1() const noexcept
    -> std::uint16_t {
  return m_mvpue_num_view_updates_minus1;
}

auto MivViewParamsUpdateIntrinsics::mvpue_view_idx(const uint16_t i) const noexcept
    -> std::uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpue_view_idx.size());
  return m_mvpue_view_idx.at(i);
}

auto MivViewParamsUpdateIntrinsics::camera_intrinsics(const uint16_t i) const noexcept
    -> const CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_intrinsics.size());
  return m_camera_intrinsics.at(i);
}

auto MivViewParamsUpdateIntrinsics::camera_intrinsics(const uint16_t i) noexcept
    -> CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_intrinsics.size());
  return m_camera_intrinsics.at(i);
}

auto MivViewParamsUpdateIntrinsics::mvpue_num_view_updates_minus1(const uint16_t value) noexcept
    -> MivViewParamsUpdateIntrinsics & {
  m_mvpue_num_view_updates_minus1 = value;
  m_mvpue_view_idx.resize(m_mvpue_num_view_updates_minus1 + 1);
  m_camera_intrinsics.resize(m_mvpue_num_view_updates_minus1 + 1);
  return *this;
}
auto MivViewParamsUpdateIntrinsics::mvpue_view_idx(const uint16_t i, const uint16_t value) noexcept
    -> MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(i < m_mvpue_view_idx.size());
  m_mvpue_view_idx.at(i) = value;
  return *this;
}

auto operator<<(ostream &stream, const MivViewParamsUpdateIntrinsics &x) -> ostream & {
  stream << "mvpue_num_view_updates_minus1=" << x.mvpue_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpue_num_view_updates_minus1(); ++i) {
    stream << "mvpue_view_idx[ " << i << " ]=" << x.mvpue_view_idx(i) << '\n';
    x.camera_intrinsics(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateIntrinsics::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpue_num_view_updates_minus1());
  for (uint16_t i = 0; i <= mvpue_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpue_view_idx(i));
    camera_intrinsics(i).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateIntrinsics::decodeFrom(InputBitstream &bitstream)
    -> MivViewParamsUpdateIntrinsics {
  auto x = MivViewParamsUpdateIntrinsics{};
  x.mvpue_num_view_updates_minus1(bitstream.getUint16());
  for (uint16_t i = 0; i <= x.mvpue_num_view_updates_minus1(); ++i) {
    x.mvpue_view_idx(i, bitstream.getUint16());
    x.camera_intrinsics(i) = CameraIntrinsics::decodeFrom(bitstream);
  }
  return x;
}

auto MivViewParamsUpdateIntrinsics::operator==(
    const MivViewParamsUpdateIntrinsics &other) const noexcept -> bool {
  return m_mvpue_num_view_updates_minus1 == other.m_mvpue_num_view_updates_minus1 &&
         std::equal(m_mvpue_view_idx.begin(), m_mvpue_view_idx.end(),
                    other.m_mvpue_view_idx.begin()) &&
         m_camera_intrinsics == other.m_camera_intrinsics;
}

auto MivViewParamsUpdateIntrinsics::operator!=(
    const MivViewParamsUpdateIntrinsics &other) const noexcept -> bool {
  return !operator==(other);
}

} // namespace TMIV::MivBitstream
