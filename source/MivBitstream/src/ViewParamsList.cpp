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

#include <TMIV/MivBitstream/ViewParamsList.h>

#include <TMIV/Common/Common.h>
#include <TMIV/Common/verify.h>

#include <stdexcept>

using namespace std::string_literals;

namespace TMIV::MivBitstream {
auto Pose::printTo(std::ostream &stream, std::uint16_t viewId) const -> std::ostream & {
  fmt::print(stream, "position[ {} ]=({}, {}, {})\n", viewId, position.x(), position.y(),
             position.z());
  fmt::print(stream, "orientation[ {} ]={} + i {} + j {} + k {}\n", viewId, orientation.w(),
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

  pose.orientation.x() = std::ldexp(static_cast<float>(ce.ce_view_quat_x()), -30);
  pose.orientation.y() = std::ldexp(static_cast<float>(ce.ce_view_quat_y()), -30);
  pose.orientation.z() = std::ldexp(static_cast<float>(ce.ce_view_quat_z()), -30);

  // Use fixed-point arithmetic for r2, because float only has 24 bits for the fraction.
  const auto r2 = Common::sqr<int64_t>(ce.ce_view_quat_x()) +
                  Common::sqr<int64_t>(ce.ce_view_quat_y()) +
                  Common::sqr<int64_t>(ce.ce_view_quat_z());
  static constexpr auto r2max = 0x1000'0000'0000'0000;
  VERIFY_MIVBITSTREAM(r2 <= r2max);

  pose.orientation.w() = std::sqrt(std::ldexp(static_cast<float>(r2max - r2), -60));
  POSTCONDITION(0 <= pose.orientation.w());

  return pose;
}

auto Pose::encodeToCameraExtrinsics() const -> CameraExtrinsics {
  PRECONDITION(normalized(orientation));

  auto ce = CameraExtrinsics{};

  ce.ce_view_pos_x(position.x());
  ce.ce_view_pos_y(position.y());
  ce.ce_view_pos_z(position.z());

  // Truncate x, y and z to guarantee x^2 + y^2 + z^2 <= 1 after reconstruction.
  ce.ce_view_quat_x(static_cast<int32_t>(std::ldexp(orientation.x(), 30)));
  ce.ce_view_quat_y(static_cast<int32_t>(std::ldexp(orientation.y(), 30)));
  ce.ce_view_quat_z(static_cast<int32_t>(std::ldexp(orientation.z(), 30)));

  return ce;
}

auto ViewParams::printTo(std::ostream &stream, uint16_t viewId) const -> std::ostream & {
  if (!name.empty()) {
    stream << "name[ " << viewId << " ]=\"" << name << "\"  # informative\n";
  }

  pose.printTo(stream, viewId);
  ci.printTo(stream, viewId);
  dq.printTo(stream, viewId);

  stream << "hasOccupancy[ " << viewId << "]=" << std::boolalpha << hasOccupancy
         << "  # encoder-internal\n";

  stream << "nbMpiLayers[ " << viewId << " ]=\"" << nbMpiLayers << "\"  # encoder-internal\n";

  if (pp) {
    pp->printTo(stream, viewId);
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
  pose.orientation = euler2quat(Common::radperdeg * node.require("Rotation").asVec<float, 3>());

  const auto depthRange = node.require("Depth_range").asVec<float, 2>();
  dq.dq_norm_disp_low(1.F / depthRange.y());
  dq.dq_norm_disp_high(1.F / depthRange.x());

  if (const auto &subnode = node.optional("HasInvalidDepth")) {
    hasOccupancy = subnode.as<bool>();
  }

  auto proj = node.require("Projection").as<std::string>();
  if (proj == "Equirectangular") {
    const auto phiRange = Common::radperdeg * node.require("Hor_range").asVec<float, 2>();
    const auto thetaRange = Common::radperdeg * node.require("Ver_range").asVec<float, 2>();

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
    nbMpiLayers = subnode.as<int>();
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
    root["Hor_range"] = Array{Json{Common::degperrad * ci.ci_erp_phi_min()},
                              Json{Common::degperrad * ci.ci_erp_phi_max()}};
    root["Ver_range"] = Array{Json{Common::degperrad * ci.ci_erp_theta_min()},
                              Json{Common::degperrad * ci.ci_erp_theta_max()}};
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

ViewParamsList::ViewParamsList(std::vector<ViewParams> viewParamsList)
    : std::vector<ViewParams>{std::move(viewParamsList)} {}

auto ViewParamsList::viewSizes() const -> Common::SizeVector {
  Common::SizeVector sizes;
  sizes.reserve(size());
  transform(begin(), end(), back_inserter(sizes),
            [](const ViewParams &viewParams) { return viewParams.ci.projectionPlaneSize(); });
  return sizes;
}

auto ViewParamsList::viewNames() const -> std::vector<std::string> {
  auto names = std::vector<std::string>(size());
  std::transform(cbegin(), cend(), names.begin(),
                 [](const ViewParams &viewParams) { return viewParams.name; });
  return names;
}

auto operator<<(std::ostream &stream, const ViewParamsList &viewParamsList) -> std::ostream & {
  for (size_t viewId = 0; viewId < viewParamsList.size(); ++viewId) {
    viewParamsList[viewId].printTo(stream, static_cast<uint16_t>(viewId));
  }
  return stream;
}

auto ViewParamsList::operator==(const ViewParamsList &other) const -> bool {
  return equal(begin(), end(), other.begin(), other.end());
}

auto ViewParamsList::loadFromJson(const Common::Json &node, const std::vector<std::string> &names)
    -> ViewParamsList {
  ViewParamsList result;
  const auto &a = node.as<Common::Json::Array>();
  for (const auto &name : names) {
    for (size_t i = 0; i != a.size(); ++i) {
      if (name == a[i].require("Name").as<std::string>()) {
        result.push_back(ViewParams{a[i]});
        break;
      }
    }
  }
  if (result.size() != names.size()) {
    throw std::runtime_error("Could not find all requested camera names in the metadata JSON file");
  }
  return result;
}
} // namespace TMIV::MivBitstream
