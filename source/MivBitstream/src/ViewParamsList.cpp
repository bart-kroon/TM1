/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ISO/IEC
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

#include <TMIV/MivBitstream/ViewParamsList.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/Quaternion.h>
#include <TMIV/Common/verify.h>

#include <regex>
#include <stdexcept>

using namespace std::string_literals;

namespace TMIV::MivBitstream {
auto Pose::printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream & {
  fmt::print(stream, "position[ {} ]=({}, {}, {})\n", viewIdx, position.x(), position.y(),
             position.z());
  fmt::print(stream, "orientation[ {} ]={} + i {} + j {} + k {}\n", viewIdx, orientation.w(),
             orientation.x(), orientation.y(), orientation.z());
  return stream;
}

auto Pose::operator==(const Pose &other) const -> bool {
  return position == other.position && orientation == other.orientation;
}

auto Pose::decodeFrom(const CameraExtrinsics &ce) -> Pose {
  auto pose = Pose{};

  pose.position.x() = ce.ce_view_pos_x();
  pose.position.y() = ce.ce_view_pos_y();
  pose.position.z() = ce.ce_view_pos_z();

  pose.orientation.x() = std::ldexp(ce.ce_view_quat_x(), -30);
  pose.orientation.y() = std::ldexp(ce.ce_view_quat_y(), -30);
  pose.orientation.z() = std::ldexp(ce.ce_view_quat_z(), -30);
  pose.orientation.w() = 0.;
  pose.orientation.w() = std::sqrt(std::max(0., 1. - norm2(pose.orientation)));

  return pose;
}

auto Pose::encodeToCameraExtrinsics() const -> CameraExtrinsics {
  auto ce = CameraExtrinsics{};

  ce.ce_view_pos_x(position.x());
  ce.ce_view_pos_y(position.y());
  ce.ce_view_pos_z(position.z());

  const auto quantize = [](const auto x) {
    return Common::assertDownCast<int32_t>(std::lround(std::ldexp(x, 30)));
  };

  const auto q = normalize(orientation);

  // NOTE(#335): Rotation p -> q -> qpq* / qq* is not affected by any non-zero scalar multiplication
  // but to avoid flips for -0, it is tested if w would be negative when quantized. This can be
  // written out as w < -eps but then an error could be made in the calculation of eps for instance
  // when the quantification equation is changed later.
  const auto sign = 0 <= quantize(q.w()) ? 1 : -1;

  ce.ce_view_quat_x(sign * quantize(q.x()));
  ce.ce_view_quat_y(sign * quantize(q.y()));
  ce.ce_view_quat_z(sign * quantize(q.z()));

  return ce;
}

auto ViewParams::printTo(std::ostream &stream, uint16_t viewIdx) const -> std::ostream & {
  fmt::print(stream, "viewId[ {} ]={}\n", viewIdx, viewId);
  if (!name.empty()) {
    stream << "name[ " << viewIdx << " ]=\"" << name << "\"  # informative\n";
  }

  pose.printTo(stream, viewIdx);
  ci.printTo(stream, viewIdx);
  dq.printTo(stream, viewIdx);

  stream << "hasOccupancy[ " << viewIdx << "]=" << std::boolalpha << hasOccupancy
         << "  # encoder-internal\n";

  stream << "nbMpiLayers[ " << viewIdx << " ]=\"" << nbMpiLayers << "\"  # encoder-internal\n";

  if (pp) {
    pp->printTo(stream, viewIdx);
  }
  return stream;
}

auto ViewParams::operator==(const ViewParams &other) const -> bool {
  return ci == other.ci && pose == other.pose && dq == other.dq && pp == other.pp;
}

ViewParams::ViewParams(const Common::Json &node) {
  name = node.require("Name").as<std::string>();

  const auto resolution = node.require("Resolution").asVec<int32_t, 2>();
  ci.ci_projection_plane_width_minus1(resolution.x() - 1);
  ci.ci_projection_plane_height_minus1(resolution.y() - 1);

  pose.position = node.require("Position").asVec<float, 3>();
  pose.orientation = eulerDeg2quat(node.require("Rotation").asVec<float, 3>());

  const auto depthRange = node.require("Depth_range").asVec<float, 2>();
  dq.dq_norm_disp_low(1.F / depthRange.y());
  dq.dq_norm_disp_high(1.F / depthRange.x());

  if (const auto &subnode = node.optional("HasInvalidDepth")) {
    hasOccupancy = subnode.as<bool>();
  }

  auto proj = node.require("Projection").as<std::string>();
  if (proj == "Equirectangular") {
    const auto phiRange = node.require("Hor_range").asVec<float, 2>();
    const auto thetaRange = node.require("Ver_range").asVec<float, 2>();

    ci.ci_cam_type(CiCamType::equirectangular);
    ci.ci_erp_phi_min(phiRange.x());
    ci.ci_erp_phi_max(phiRange.y());
    ci.ci_erp_theta_min(thetaRange.x());
    ci.ci_erp_theta_max(thetaRange.y());

  } else if (proj == "Perspective") {
    const auto focal = node.require("Focal").asVec<float, 2>();
    const auto center = node.require("Principle_point").asVec<float, 2>();

    ci.ci_cam_type(CiCamType::perspective);
    ci.ci_perspective_focal_hor(focal.x());
    ci.ci_perspective_focal_ver(focal.y());
    ci.ci_perspective_center_hor(center.x());
    ci.ci_perspective_center_ver(center.y());

  } else if (proj == "Orthographic") {
    ci.ci_cam_type(CiCamType::orthographic);
    ci.ci_ortho_width(node.require("OrthoWidth").as<float>());
    ci.ci_ortho_width(node.require("OrthoHeight").as<float>());

  } else {
    throw std::runtime_error("Unknown projection type in metadata JSON file");
  }

  if (auto subnode = node.optional("nbMpiLayers")) {
    nbMpiLayers = subnode.as<int32_t>();
  }
}

ViewParams::operator Common::Json() const {
  using Common::Json;
  using Array = Json::Array;

  auto root = Json::Object{};

  root["Name"s] = name;
  root["Resolution"s] = Array{Json{ci.ci_projection_plane_width_minus1() + 1},
                              Json{ci.ci_projection_plane_height_minus1() + 1}};
  root["Position"s] =
      Array{Json{pose.position.x()}, Json{pose.position.y()}, Json{pose.position.z()}};

  const auto euler = quat2euler(Common::QuatD{pose.orientation});
  root["Rotation"s] = Array{Json{Common::rad2deg(euler.x())}, Json{Common::rad2deg(euler.y())},
                            Json{Common::rad2deg(euler.z())}};

  root["Depth_range"s] = Array{Json{1. / dq.dq_norm_disp_high()}, Json{1. / dq.dq_norm_disp_low()}};

  root["HasInvalidDepth"] = hasOccupancy;

  if (ci.ci_cam_type() == CiCamType::equirectangular) {
    root["Projection"] = "Equirectangular";
    root["Hor_range"] = Array{Json{ci.ci_erp_phi_min()}, Json{ci.ci_erp_phi_max()}};
    root["Ver_range"] = Array{Json{ci.ci_erp_theta_min()}, Json{ci.ci_erp_theta_max()}};
  } else if (ci.ci_cam_type() == CiCamType::perspective) {
    root["Projection"] = "Perspective";
    root["Focal"] = Array{Json{ci.ci_perspective_focal_hor()}, Json{ci.ci_perspective_focal_ver()}};
    root["Principle_point"] =
        Array{Json{ci.ci_perspective_center_hor()}, Json{ci.ci_perspective_center_ver()}};
  } else {
    root["Projection"] = "Orthographic";
    root["OrthoWidth"] = ci.ci_ortho_width();
    root["OrthoHeight"] = ci.ci_ortho_height();
  }

  return Json{root};
}

auto ViewParams::viewRoot() const -> bool { return pp && pp->pp_is_root_flag(); }

auto ViewParams::viewNumParents() const -> uint16_t {
  if (pp && !pp->pp_is_root_flag()) {
    return Common::assertDownCast<uint16_t>(pp->pp_num_parent_minus1() + 1);
  }
  return {};
}

auto ViewParams::viewParentIdx(uint16_t i) const -> uint16_t {
  if (pp && !pp->pp_is_root_flag()) {
    return pp->pp_parent_idx(i);
  }
  return {};
}

void ViewParamsList::constructViewIdIndex() {
  // Test for duplicate ID's
  auto usedSlot = std::vector<bool>(maxViewIdValue() + size_t{1}, false);

  m_viewIdIndex.assign(usedSlot.size(), UINT16_MAX);
  auto viewIdx = uint16_t{};

  for (const auto &vp : *this) {
    VERIFY(!usedSlot[vp.viewId.m_value] && "Duplicate view ID");
    usedSlot[vp.viewId.m_value] = true;

    m_viewIdIndex[vp.viewId.m_value] = viewIdx++;
  }
}

void ViewParamsList::assignViewIds(vector<uint16_t> sourceCameraIds) {
  auto viewIdx = uint16_t{};

  for (auto &vp : *this) {
    if (sourceCameraIds.empty()) {
      vp.viewId = ViewId{viewIdx++};
    } else {
      VERIFY(viewIdx < sourceCameraIds.size());
      vp.viewId = ViewId{sourceCameraIds[viewIdx++]};
    }
  }
}

auto ViewParamsList::maxViewIdValue() const noexcept -> uint16_t {
  PRECONDITION(!empty());
  return std::max_element(cbegin(), cend(),
                          [](const auto &vp1, const auto &vp2) { return vp1.viewId < vp2.viewId; })
      ->viewId.m_value;
}
} // namespace TMIV::MivBitstream
