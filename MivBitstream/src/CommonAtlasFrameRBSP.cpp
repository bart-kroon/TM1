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

#include <TMIV/MivBitstream/CommonAtlasFrameRBSP.h>

#include <TMIV/MivBitstream/verify.h>

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
  case MvpUpdateMode::VPL_UPD_DQ:
    return stream << "VPL_UPD_DQ";
  case MvpUpdateMode::VPL_ALL:
    return stream << "VPL_ALL";
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

PruningParent::PruningParent(vector<uint16_t> pp_parent_id) : m_pp_parent_id{move(pp_parent_id)} {}

auto PruningParent::pp_is_root_flag() const noexcept -> bool { return m_pp_parent_id.empty(); }

auto PruningParent::pp_num_parent_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(!pp_is_root_flag());
  return uint16_t(m_pp_parent_id.size() - 1);
}

auto PruningParent::pp_parent_id(uint16_t i) const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_pp_parent_id.size());
  return m_pp_parent_id[i];
}

auto PruningParent::pp_parent_id(std::uint16_t i, std::uint16_t value) noexcept -> PruningParent & {
  VERIFY_MIVBITSTREAM(i < m_pp_parent_id.size());
  m_pp_parent_id[i] = value;
  return *this;
}

auto PruningParent::printTo(ostream &stream, uint16_t viewId) const -> ostream & {
  stream << "pp_is_root_flag[ " << viewId << " ]=" << boolalpha << pp_is_root_flag() << '\n';
  if (!pp_is_root_flag()) {
    stream << "pp_num_parent_minus1[ " << viewId << " ]=" << pp_num_parent_minus1() << '\n';
    for (auto i = 0; i <= pp_num_parent_minus1(); ++i) {
      stream << "pp_parent_id[ " << viewId << " ][ " << i << " ]=" << pp_parent_id(i) << '\n';
    }
  }
  return stream;
}

auto PruningParent::operator==(const PruningParent &other) const noexcept -> bool {
  return m_pp_parent_id == other.m_pp_parent_id;
}

auto PruningParent::operator!=(const PruningParent &other) const noexcept -> bool {
  return !operator==(other);
}

auto PruningParent::decodeFrom(InputBitstream &bitstream, uint16_t mvp_num_views_minus1)
    -> PruningParent {
  const auto pp_is_root_flag = bitstream.getFlag();
  if (pp_is_root_flag) {
    return {};
  }

  const auto pp_num_parent_minus1 = bitstream.getUVar<size_t>(mvp_num_views_minus1);
  auto x = vector<uint16_t>(pp_num_parent_minus1 + 1);

  for (uint16_t &i : x) {
    i = bitstream.getUVar<uint16_t>(mvp_num_views_minus1 + uint64_t(1));
  }

  return PruningParent{x};
}

void PruningParent::encodeTo(OutputBitstream &bitstream, uint16_t mvp_num_views_minus1) const {
  bitstream.putFlag(pp_is_root_flag());
  if (!pp_is_root_flag()) {
    bitstream.putUVar(pp_num_parent_minus1(), mvp_num_views_minus1);
    for (uint16_t i = 0; i <= pp_num_parent_minus1(); ++i) {
      bitstream.putUVar(pp_parent_id(i), mvp_num_views_minus1 + uint64_t(1));
    }
  }
}

auto MivViewParamsList::mvp_num_views_minus1() const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(!m_camera_extrinsics.empty());
  return uint16_t(m_camera_extrinsics.size() - 1);
}

auto MivViewParamsList::mvp_view_enabled_present_flag() const noexcept -> bool {
  return m_mvp_view_enabled_present_flag;
}

auto MivViewParamsList::mvp_view_enabled_in_atlas_flag(uint8_t atlasIdx,
                                                       uint16_t viewIdx) const noexcept -> bool {
  VERIFY_MIVBITSTREAM(mvp_view_enabled_present_flag());
  VERIFY_MIVBITSTREAM(atlasIdx < m_viewInAtlas.size());
  VERIFY_MIVBITSTREAM(viewIdx <= mvp_num_views_minus1());
  return m_viewInAtlas[atlasIdx][viewIdx].enabled;
}

auto MivViewParamsList::mvp_view_complete_in_atlas_flag(uint8_t atlasIdx,
                                                        uint16_t viewIdx) const noexcept -> bool {
  VERIFY_MIVBITSTREAM(mvp_view_enabled_in_atlas_flag(atlasIdx, viewIdx));
  VERIFY_MIVBITSTREAM(m_viewInAtlas[atlasIdx][viewIdx].complete.has_value());
  return *m_viewInAtlas[atlasIdx][viewIdx].complete;
}

auto MivViewParamsList::mvp_view_id(uint16_t viewIdx) const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(viewIdx <= mvp_num_views_minus1());
  return m_mvp_view_id[viewIdx];
}

auto MivViewParamsList::camera_extrinsics(uint16_t viewId) const noexcept
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

auto MivViewParamsList::pruning_parent(uint16_t viewId) const noexcept -> const PruningParent & {
  VERIFY_MIVBITSTREAM(mvp_pruning_graph_params_present_flag());
  VERIFY_MIVBITSTREAM(viewId < m_pruning_parent.size());
  return m_pruning_parent[viewId];
}

auto MivViewParamsList::mvp_num_views_minus1(uint16_t value) noexcept -> MivViewParamsList & {
  for (auto &x : m_viewInAtlas) {
    x.resize(size_t(value) + 1);
  }
  m_camera_extrinsics.resize(size_t(value) + 1);
  return *this;
}

auto MivViewParamsList::mvp_view_enabled_present_flag(bool value) noexcept -> MivViewParamsList & {
  m_mvp_view_enabled_present_flag = value;
  return *this;
}

auto MivViewParamsList::mvp_view_enabled_in_atlas_flag(uint8_t atlasIdx, uint16_t viewIdx,
                                                       bool value) noexcept -> MivViewParamsList & {
  mvp_view_enabled_present_flag(true);
  VERIFY_MIVBITSTREAM(viewIdx <= mvp_num_views_minus1());
  while (atlasIdx >= m_viewInAtlas.size()) {
    m_viewInAtlas.emplace_back(mvp_num_views_minus1() + 1);
  }
  m_viewInAtlas[atlasIdx][viewIdx].enabled = value;
  return *this;
}

auto MivViewParamsList::mvp_view_complete_in_atlas_flag(uint8_t atlasIdx, uint16_t viewIdx,
                                                        bool value) noexcept
    -> MivViewParamsList & {
  VERIFY_MIVBITSTREAM(mvp_view_enabled_in_atlas_flag(atlasIdx, viewIdx));
  m_viewInAtlas[atlasIdx][viewIdx].complete = value;
  return *this;
}

auto MivViewParamsList::mvp_explicit_view_id_flag(bool value) noexcept -> MivViewParamsList & {
  m_mvp_explicit_view_id_flag = value;
  return *this;
}

auto MivViewParamsList::mvp_view_id(uint16_t viewIdx, uint16_t viewId) noexcept
    -> MivViewParamsList & {
  VERIFY_MIVBITSTREAM(mvp_explicit_view_id_flag());
  if (m_mvp_view_id.size() < mvp_num_views_minus1() + size_t(1)) {
    m_mvp_view_id.resize(mvp_num_views_minus1() + size_t(1));
  }
  VERIFY_MIVBITSTREAM(viewIdx <= mvp_num_views_minus1());
  m_mvp_view_id[viewIdx] = viewId;
  return *this;
}

auto MivViewParamsList::mvp_intrinsic_params_equal_flag(bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_intrinsic_params_equal_flag = value;
  m_camera_intrinsics.resize(value ? 1U : m_camera_extrinsics.size());
  return *this;
}

auto MivViewParamsList::mvp_depth_quantization_params_equal_flag(bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_depth_quantization_params_equal_flag = value;
  m_depth_quantization.resize(value ? 1U : m_camera_extrinsics.size());
  return *this;
}

auto MivViewParamsList::mvp_pruning_graph_params_present_flag(bool value) noexcept
    -> MivViewParamsList & {
  m_mvp_pruning_graph_params_present_flag = value;
  m_pruning_parent.resize(value ? m_camera_extrinsics.size() : 0U);
  return *this;
}

auto MivViewParamsList::camera_extrinsics(uint16_t viewId) noexcept -> CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_extrinsics.size());
  return m_camera_extrinsics[viewId];
}

auto MivViewParamsList::camera_intrinsics(uint16_t viewId) noexcept -> CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(viewId < m_camera_intrinsics.size());
  return m_camera_intrinsics[viewId];
}

auto MivViewParamsList::depth_quantization(uint16_t viewId) noexcept -> DepthQuantization & {
  VERIFY_MIVBITSTREAM(viewId < m_depth_quantization.size());
  return m_depth_quantization[viewId];
}

auto MivViewParamsList::pruning_parent(uint16_t viewId) noexcept -> PruningParent & {
  VERIFY_MIVBITSTREAM(viewId < m_pruning_parent.size());
  return m_pruning_parent[viewId];
}

auto operator<<(ostream &stream, const MivViewParamsList &x) -> ostream & {
  stream << "mvp_num_views_minus1=" << x.mvp_num_views_minus1() << '\n';
  stream << "mvp_view_enabled_present_flag=" << boolalpha << x.mvp_view_enabled_present_flag()
         << '\n';

  if (x.mvp_view_enabled_present_flag()) {
    for (size_t a = 0; a < x.m_viewInAtlas.size(); ++a) {
      for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
        stream << "mvp_view_enabled_in_atlas_flag[ " << a << " ][ " << v << " ]=" << boolalpha
               << x.mvp_view_enabled_in_atlas_flag(uint8_t(a), v) << '\n';
        if (x.mvp_view_enabled_in_atlas_flag(uint8_t(a), v)) {
          stream << "mvp_view_complete_in_atlas_flag[ " << a << " ][ " << v << " ]=" << boolalpha
                 << x.mvp_view_complete_in_atlas_flag(uint8_t(a), v) << '\n';
        }
      }
    }
  }

  stream << "mvp_explicit_view_id_flag=" << boolalpha << x.mvp_explicit_view_id_flag() << '\n';
  if (x.mvp_explicit_view_id_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      stream << "mvp_view_id[ " << v << " ]=" << x.mvp_view_id(v) << '\n';
    }
  }

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
      x.pruning_parent(v).printTo(stream, v);
    }
  }
  return stream;
}

auto MivViewParamsList::operator==(const MivViewParamsList &other) const noexcept -> bool {
  return m_viewInAtlas == other.m_viewInAtlas &&
         m_mvp_explicit_view_id_flag == other.m_mvp_explicit_view_id_flag &&
         m_mvp_view_id == other.m_mvp_view_id && m_camera_extrinsics == other.m_camera_extrinsics &&
         m_mvp_intrinsic_params_equal_flag == other.m_mvp_intrinsic_params_equal_flag &&
         m_camera_intrinsics == other.m_camera_intrinsics &&
         m_mvp_depth_quantization_params_equal_flag ==
             other.m_mvp_depth_quantization_params_equal_flag &&
         m_depth_quantization == other.m_depth_quantization &&
         m_mvp_pruning_graph_params_present_flag == other.m_mvp_pruning_graph_params_present_flag &&
         m_pruning_parent == other.m_pruning_parent;
}

auto MivViewParamsList::operator!=(const MivViewParamsList &other) const noexcept -> bool {
  return !operator==(other);
}

auto MivViewParamsList::decodeFrom(InputBitstream &bitstream, const V3cParameterSet &vps)
    -> MivViewParamsList {
  auto x = MivViewParamsList{};

  x.mvp_num_views_minus1(bitstream.getUint16());
  x.mvp_view_enabled_present_flag(bitstream.getFlag());

  if (x.mvp_view_enabled_present_flag()) {
    for (uint8_t a = 0; a <= vps.vps_atlas_count_minus1(); ++a) {
      for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
        x.mvp_view_enabled_in_atlas_flag(a, v, bitstream.getFlag());
        if (x.mvp_view_enabled_in_atlas_flag(a, v)) {
          x.mvp_view_complete_in_atlas_flag(a, v, bitstream.getFlag());
        }
      }
    }
  }

  x.mvp_explicit_view_id_flag(bitstream.getFlag());
  if (x.mvp_explicit_view_id_flag()) {
    for (uint16_t v = 0; v <= x.mvp_num_views_minus1(); ++v) {
      x.mvp_view_id(v, bitstream.getUint16());
    }
  }

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
      x.pruning_parent(v) = PruningParent::decodeFrom(bitstream, x.mvp_num_views_minus1());
    }
  }
  return x;
}

void MivViewParamsList::encodeTo(OutputBitstream &bitstream, const V3cParameterSet &vps) const {
  bitstream.putUint16(mvp_num_views_minus1());
  bitstream.putFlag(mvp_view_enabled_present_flag());

  if (mvp_view_enabled_present_flag()) {
    for (uint8_t a = 0; a <= vps.vps_atlas_count_minus1(); ++a) {
      for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
        bitstream.putFlag(mvp_view_enabled_in_atlas_flag(a, v));
        if (mvp_view_enabled_in_atlas_flag(a, v)) {
          bitstream.putFlag(mvp_view_complete_in_atlas_flag(a, v));
        }
      }
    }
  }

  bitstream.putFlag(mvp_explicit_view_id_flag());
  if (mvp_explicit_view_id_flag()) {
    for (uint16_t v = 0; v <= mvp_num_views_minus1(); ++v) {
      bitstream.putUint16(mvp_view_id(v));
    }
  }

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
      pruning_parent(v).encodeTo(bitstream, mvp_num_views_minus1());
    }
  }
}

auto CommonAtlasFrameRBSP::miv_view_params_list() const noexcept -> const MivViewParamsList & {
  VERIFY_MIVBITSTREAM(caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_INITLIST);
  VERIFY_MIVBITSTREAM(m_miv_view_params_list.has_value());
  return *m_miv_view_params_list;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_extrinsics() const noexcept
    -> const MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_ALL ||
                      caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_UPD_EXT);
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_extrinsics.has_value());
  return *m_miv_view_params_update_extrinsics;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_intrinsics() const noexcept
    -> const MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_ALL ||
                      caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_UPD_INT);
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_intrinsics.has_value());
  return *m_miv_view_params_update_intrinsics;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_depth_quantization() const noexcept
    -> const MivViewParamsUpdateDepthQuantization & {
  VERIFY_MIVBITSTREAM(caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_ALL ||
                      caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_UPD_DQ);
  VERIFY_MIVBITSTREAM(m_miv_view_params_update_depth_quantization.has_value());
  return *m_miv_view_params_update_depth_quantization;
}

auto CommonAtlasFrameRBSP::miv_view_params_list() noexcept -> MivViewParamsList & {
  VERIFY_MIVBITSTREAM(caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_INITLIST);
  if (!m_miv_view_params_list) {
    m_miv_view_params_list = MivViewParamsList{};
  }
  return *m_miv_view_params_list;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_extrinsics() noexcept
    -> MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_ALL ||
                      caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_UPD_EXT);
  if (!m_miv_view_params_update_extrinsics) {
    m_miv_view_params_update_extrinsics = MivViewParamsUpdateExtrinsics{};
  }
  return *m_miv_view_params_update_extrinsics;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_intrinsics() noexcept
    -> MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_ALL ||
                      caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_UPD_INT);
  if (!m_miv_view_params_update_intrinsics) {
    m_miv_view_params_update_intrinsics = MivViewParamsUpdateIntrinsics{};
  }
  return *m_miv_view_params_update_intrinsics;
}

auto CommonAtlasFrameRBSP::miv_view_params_update_depth_quantization() noexcept
    -> MivViewParamsUpdateDepthQuantization & {
  VERIFY_MIVBITSTREAM(caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_ALL ||
                      caf_miv_view_params_list_update_mode() == MvpUpdateMode::VPL_UPD_DQ);
  if (!m_miv_view_params_update_depth_quantization) {
    m_miv_view_params_update_depth_quantization = MivViewParamsUpdateDepthQuantization{};
  }
  return *m_miv_view_params_update_depth_quantization;
}

auto CommonAtlasFrameRBSP::cafExtensionData() const noexcept -> const vector<bool> & {
  VERIFY_V3CBITSTREAM(caf_extension_8bits() != 0);
  VERIFY_V3CBITSTREAM(m_cafExtensionData.has_value());
  return *m_cafExtensionData;
}

auto CommonAtlasFrameRBSP::caf_extension_8bits(std::uint8_t value) noexcept
    -> CommonAtlasFrameRBSP & {
  VERIFY_V3CBITSTREAM(caf_extension_present_flag());
  m_caf_extension_8bits = value;
  return *this;
}

auto CommonAtlasFrameRBSP::cafExtensionData(std::vector<bool> value) noexcept
    -> CommonAtlasFrameRBSP & {
  VERIFY_V3CBITSTREAM(caf_extension_8bits() != 0);
  m_cafExtensionData = move(value);
  return *this;
}

auto operator<<(ostream &stream, const CommonAtlasFrameRBSP &x) -> ostream & {
  stream << "caf_atlas_adaptation_parameter_set_id="
         << int(x.caf_atlas_adaptation_parameter_set_id()) << '\n';
  stream << "caf_frm_order_cnt_lsb=" << x.caf_frm_order_cnt_lsb() << '\n';
  stream << "caf_miv_view_params_list_update_mode=" << x.caf_miv_view_params_list_update_mode()
         << '\n';
  switch (x.caf_miv_view_params_list_update_mode()) {
  case MvpUpdateMode::VPL_INITLIST:
    stream << x.miv_view_params_list();
    break;
  case MvpUpdateMode::VPL_UPD_EXT:
    stream << x.miv_view_params_update_extrinsics();
    break;
  case MvpUpdateMode::VPL_UPD_INT:
    stream << x.miv_view_params_update_intrinsics();
    break;
  case MvpUpdateMode::VPL_UPD_DQ:
    stream << x.miv_view_params_update_depth_quantization();
    break;
  case MvpUpdateMode::VPL_ALL:
    stream << x.miv_view_params_update_extrinsics();
    stream << x.miv_view_params_update_intrinsics();
    stream << x.miv_view_params_update_depth_quantization();
    break;
  default:
    MIVBITSTREAM_ERROR("Unknown update mode");
  }

  stream << "caf_extension_present_flag=" << boolalpha << x.caf_extension_present_flag() << '\n';
  if (x.caf_extension_present_flag()) {
    stream << "caf_extension_8bits=" << int(x.caf_extension_8bits()) << '\n';
  }
  if (x.caf_extension_8bits()) {
    for (auto bit : x.cafExtensionData()) {
      stream << "caf_extension_data_flag=" << boolalpha << bit << '\n';
    }
  }
  return stream;
}

auto CommonAtlasFrameRBSP::operator==(const CommonAtlasFrameRBSP &other) const noexcept -> bool {
  if (caf_atlas_adaptation_parameter_set_id() != other.caf_atlas_adaptation_parameter_set_id() ||
      caf_frm_order_cnt_lsb() != other.caf_frm_order_cnt_lsb() ||
      caf_miv_view_params_list_update_mode() != other.caf_miv_view_params_list_update_mode() ||
      caf_extension_present_flag() != other.caf_extension_present_flag() ||
      caf_extension_8bits() != other.caf_extension_8bits()) {
    return false;
  }

  switch (caf_miv_view_params_list_update_mode()) {
  case MvpUpdateMode::VPL_INITLIST:
    return miv_view_params_list() == other.miv_view_params_list();
  case MvpUpdateMode::VPL_UPD_EXT:
    return miv_view_params_update_extrinsics() == other.miv_view_params_update_extrinsics();
  case MvpUpdateMode::VPL_UPD_INT:
    return miv_view_params_update_intrinsics() == other.miv_view_params_update_intrinsics();
  case MvpUpdateMode::VPL_UPD_DQ:
    return miv_view_params_update_depth_quantization() ==
           other.miv_view_params_update_depth_quantization();
  case MvpUpdateMode::VPL_ALL:
    return miv_view_params_update_extrinsics() == other.miv_view_params_update_extrinsics() &&
           miv_view_params_update_intrinsics() == other.miv_view_params_update_intrinsics() &&
           miv_view_params_update_depth_quantization() ==
               other.miv_view_params_update_depth_quantization();
  default:
    MIVBITSTREAM_ERROR("Unknown update mode");
  }

  if (caf_extension_8bits() != 0 && cafExtensionData() != other.cafExtensionData()) {
    return false;
  }
  return true;
}

auto CommonAtlasFrameRBSP::operator!=(const CommonAtlasFrameRBSP &other) const noexcept -> bool {
  return !operator==(other);
}

auto CommonAtlasFrameRBSP::decodeFrom(istream &stream, const V3cParameterSet &vps,
                                      unsigned maxCommonAtlasFrmOrderCntLsb)
    -> CommonAtlasFrameRBSP {
  InputBitstream bitstream{stream};

  auto x = CommonAtlasFrameRBSP{};

  x.caf_atlas_adaptation_parameter_set_id(bitstream.getUExpGolomb<uint8_t>());
  x.caf_frm_order_cnt_lsb(bitstream.getUVar<uint16_t>(maxCommonAtlasFrmOrderCntLsb));
  x.caf_miv_view_params_list_update_mode(bitstream.readBits<MvpUpdateMode>(2));

  switch (x.caf_miv_view_params_list_update_mode()) {
  case MvpUpdateMode::VPL_INITLIST:
    x.miv_view_params_list() = MivViewParamsList::decodeFrom(bitstream, vps);
    break;
  case MvpUpdateMode::VPL_UPD_EXT:
    x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
    break;
  case MvpUpdateMode::VPL_UPD_INT:
    x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
    break;
  case MvpUpdateMode::VPL_UPD_DQ:
    x.miv_view_params_update_depth_quantization() =
        MivViewParamsUpdateDepthQuantization::decodeFrom(bitstream);
    break;
  case MvpUpdateMode::VPL_ALL:
    x.miv_view_params_update_extrinsics() = MivViewParamsUpdateExtrinsics::decodeFrom(bitstream);
    x.miv_view_params_update_intrinsics() = MivViewParamsUpdateIntrinsics::decodeFrom(bitstream);
    x.miv_view_params_update_depth_quantization() =
        MivViewParamsUpdateDepthQuantization::decodeFrom(bitstream);
    break;
  }

  x.caf_extension_present_flag(bitstream.getFlag());

  if (x.caf_extension_present_flag()) {
    x.caf_extension_8bits(bitstream.readBits<uint8_t>(8));
  }
  if (x.caf_extension_8bits() != 0) {
    auto cafExtensionData = vector<bool>{};
    while (bitstream.moreRbspData()) {
      cafExtensionData.push_back(bitstream.getFlag());
    }
    x.cafExtensionData(move(cafExtensionData));
  }
  bitstream.rbspTrailingBits();

  return x;
}

void CommonAtlasFrameRBSP::encodeTo(ostream &stream, const V3cParameterSet &vps,
                                    unsigned maxCommonAtlasFrmOrderCntLsb) const {
  OutputBitstream bitstream{stream};

  bitstream.putUExpGolomb(caf_atlas_adaptation_parameter_set_id());
  bitstream.putUVar(caf_frm_order_cnt_lsb(), maxCommonAtlasFrmOrderCntLsb);
  bitstream.writeBits(caf_miv_view_params_list_update_mode(), 2);

  switch (caf_miv_view_params_list_update_mode()) {
  case MvpUpdateMode::VPL_INITLIST:
    miv_view_params_list().encodeTo(bitstream, vps);
    break;
  case MvpUpdateMode::VPL_UPD_EXT:
    miv_view_params_update_extrinsics().encodeTo(bitstream);
    break;
  case MvpUpdateMode::VPL_UPD_INT:
    miv_view_params_update_intrinsics().encodeTo(bitstream);
    break;
  case MvpUpdateMode::VPL_UPD_DQ:
    miv_view_params_update_depth_quantization().encodeTo(bitstream);
    break;
  case MvpUpdateMode::VPL_ALL:
    miv_view_params_update_extrinsics().encodeTo(bitstream);
    miv_view_params_update_intrinsics().encodeTo(bitstream);
    miv_view_params_update_depth_quantization().encodeTo(bitstream);
    break;
  }

  bitstream.putFlag(caf_extension_present_flag());

  if (caf_extension_present_flag()) {
    bitstream.writeBits(caf_extension_8bits(), 8);
  }
  if (caf_extension_8bits() != 0) {
    for (auto bit : cafExtensionData()) {
      bitstream.putFlag(bit);
    }
  }
  bitstream.rbspTrailingBits();
}

auto MivViewParamsUpdateExtrinsics::mvpue_num_view_updates_minus1() const noexcept -> uint16_t {
  return m_mvpue_num_view_updates_minus1;
}

auto MivViewParamsUpdateExtrinsics::mvpue_view_idx(const uint16_t i) const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpue_view_idx.size());
  return m_mvpue_view_idx[i];
}

auto MivViewParamsUpdateExtrinsics::camera_extrinsics(const uint16_t i) const noexcept
    -> const CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_extrinsics.size());
  return m_camera_extrinsics[i];
}
auto MivViewParamsUpdateExtrinsics::camera_extrinsics(const uint16_t i) noexcept
    -> CameraExtrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_extrinsics.size());
  return m_camera_extrinsics[i];
}

auto MivViewParamsUpdateExtrinsics::mvpue_num_view_updates_minus1(const uint16_t value) noexcept
    -> MivViewParamsUpdateExtrinsics & {
  m_mvpue_num_view_updates_minus1 = value;
  m_mvpue_view_idx.resize(value + size_t(1));
  m_camera_extrinsics.resize(value + size_t(1));
  return *this;
}
auto MivViewParamsUpdateExtrinsics::mvpue_view_idx(const uint16_t i, const uint16_t value) noexcept
    -> MivViewParamsUpdateExtrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_extrinsics.size());
  m_mvpue_view_idx[i] = value;
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

auto MivViewParamsUpdateIntrinsics::mvpui_view_idx(const uint16_t i) const noexcept -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpui_view_idx.size());
  return m_mvpui_view_idx[i];
}

auto MivViewParamsUpdateIntrinsics::camera_intrinsics(const uint16_t i) const noexcept
    -> const CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_intrinsics.size());
  return m_camera_intrinsics[i];
}

auto MivViewParamsUpdateIntrinsics::camera_intrinsics(const uint16_t i) noexcept
    -> CameraIntrinsics & {
  VERIFY_MIVBITSTREAM(i < m_camera_intrinsics.size());
  return m_camera_intrinsics[i];
}

auto MivViewParamsUpdateIntrinsics::mvpui_num_view_updates_minus1(const uint16_t value) noexcept
    -> MivViewParamsUpdateIntrinsics & {
  m_mvpui_num_view_updates_minus1 = value;
  m_mvpui_view_idx.resize(m_mvpui_num_view_updates_minus1 + size_t(1));
  m_camera_intrinsics.resize(m_mvpui_num_view_updates_minus1 + size_t(1));
  return *this;
}

auto MivViewParamsUpdateIntrinsics::mvpui_view_idx(const uint16_t i, const uint16_t value) noexcept
    -> MivViewParamsUpdateIntrinsics & {
  VERIFY_MIVBITSTREAM(i < m_mvpui_view_idx.size());
  m_mvpui_view_idx[i] = value;
  return *this;
}

auto operator<<(ostream &stream, const MivViewParamsUpdateIntrinsics &x) -> ostream & {
  stream << "mvpui_num_view_updates_minus1=" << x.mvpui_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpui_num_view_updates_minus1(); ++i) {
    stream << "mvpui_view_idx[ " << i << " ]=" << x.mvpui_view_idx(i) << '\n';
    x.camera_intrinsics(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateIntrinsics::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpui_num_view_updates_minus1());
  for (uint16_t i = 0; i <= mvpui_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpui_view_idx(i));
    camera_intrinsics(i).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateIntrinsics::decodeFrom(InputBitstream &bitstream)
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

auto MivViewParamsUpdateDepthQuantization::mvpudq_view_idx(const uint16_t i) const noexcept
    -> uint16_t {
  VERIFY_MIVBITSTREAM(i < m_mvpudq_view_idx.size());
  return m_mvpudq_view_idx[i];
}

auto MivViewParamsUpdateDepthQuantization::depth_quantization(const uint16_t i) const noexcept
    -> const DepthQuantization & {
  VERIFY_MIVBITSTREAM(i < m_depth_quantization.size());
  return m_depth_quantization[i];
}

auto MivViewParamsUpdateDepthQuantization::depth_quantization(const uint16_t i) noexcept
    -> DepthQuantization & {
  VERIFY_MIVBITSTREAM(i < m_depth_quantization.size());
  return m_depth_quantization[i];
}

auto MivViewParamsUpdateDepthQuantization::mvpudq_num_view_updates_minus1(
    const uint16_t value) noexcept -> MivViewParamsUpdateDepthQuantization & {
  m_mvpudq_num_view_updates_minus1 = value;
  m_mvpudq_view_idx.resize(m_mvpudq_num_view_updates_minus1 + size_t(1));
  m_depth_quantization.resize(m_mvpudq_num_view_updates_minus1 + size_t(1));
  return *this;
}
auto MivViewParamsUpdateDepthQuantization::mvpudq_view_idx(const uint16_t i,
                                                           const uint16_t value) noexcept
    -> MivViewParamsUpdateDepthQuantization & {
  VERIFY_MIVBITSTREAM(i < m_mvpudq_view_idx.size());
  m_mvpudq_view_idx[i] = value;
  return *this;
}

auto operator<<(ostream &stream, const MivViewParamsUpdateDepthQuantization &x) -> ostream & {
  stream << "mvpudq_num_view_updates_minus1=" << x.mvpudq_num_view_updates_minus1() << '\n';
  for (uint16_t i = 0; i <= x.mvpudq_num_view_updates_minus1(); ++i) {
    stream << "mvpudq_view_idx[ " << i << " ]=" << x.mvpudq_view_idx(i) << '\n';
    x.depth_quantization(i).printTo(stream, i);
  }
  return stream;
}

void MivViewParamsUpdateDepthQuantization::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putUint16(mvpudq_num_view_updates_minus1());
  for (uint16_t i = 0; i <= mvpudq_num_view_updates_minus1(); ++i) {
    bitstream.putUint16(mvpudq_view_idx(i));
    depth_quantization(i).encodeTo(bitstream);
  }
}

auto MivViewParamsUpdateDepthQuantization::decodeFrom(InputBitstream &bitstream)
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
} // namespace TMIV::MivBitstream
