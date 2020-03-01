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

#include <TMIV/MivBitstream/ViewParamsList.h>

#include <TMIV/Common/Common.h>

#include "verify.h"

#include <stdexcept>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto ViewParams::printTo(ostream &stream, uint16_t viewId) const -> ostream & {
  if (!name.empty()) {
    stream << "name[ " << viewId << " ]=\"" << name << "\"  # informative\n";
  }

  ce.printTo(stream, viewId);
  ci.printTo(stream, viewId);
  dq.printTo(stream, viewId);

  stream << "hasOccupancy[ " << viewId << "]=" << boolalpha << hasOccupancy
         << "  # encoder-internal\n";

  if (pc) {
    pc->printTo(stream, viewId);
  }
  return stream;
}

auto ViewParams::operator==(const ViewParams &other) const -> bool {
  return ci == other.ci && ce == other.ce && dq == other.dq && pc == other.pc;
}

auto ViewParams::loadFromJson(const Json &node) -> ViewParams {
  auto x = ViewParams{};
  x.name = node.require("Name").asString();

  const auto resolution = node.require("Resolution").asIntVector<2>();
  x.ci.ci_projection_plane_width_minus1(resolution.x() - 1);
  x.ci.ci_projection_plane_height_minus1(resolution.y() - 1);

  x.ce.position(node.require("Position").asFloatVector<3>());
  x.ce.rotation(euler2quat(radperdeg * node.require("Rotation").asFloatVector<3>()));

  const auto depthRange = node.require("Depth_range").asFloatVector<2>();
  constexpr auto kilometer = 1000.F;
  x.dq.dq_norm_disp_low(depthRange.y() < kilometer ? 1.F / depthRange.y() : 0.F);
  x.dq.dq_norm_disp_high(depthRange.x() < kilometer ? 1.F / depthRange.x() : 0.F);

  if (auto subnode = node.optional("HasInvalidDepth"); subnode) {
    x.hasOccupancy = subnode.asBool();
  }

  auto proj = node.require("Projection").asString();
  if (proj == "Equirectangular") {
    const auto phiRange = radperdeg * node.require("Hor_range").asFloatVector<2>();
    const auto thetaRange = radperdeg * node.require("Ver_range").asFloatVector<2>();

    x.ci.ci_cam_type(CiCamType::equirectangular);
    x.ci.ci_erp_phi_min(phiRange.x());
    x.ci.ci_erp_phi_max(phiRange.y());
    x.ci.ci_erp_theta_min(thetaRange.x());
    x.ci.ci_erp_theta_max(thetaRange.y());

  } else if (proj == "Perspective") {
    const auto focal = node.require("Focal").asFloatVector<2>();
    const auto center = node.require("Principle_point").asFloatVector<2>();

    x.ci.ci_cam_type(CiCamType::perspective);
    x.ci.ci_perspective_focal_hor(focal.x());
    x.ci.ci_perspective_focal_ver(focal.y());
    x.ci.ci_perspective_center_hor(center.x());
    x.ci.ci_perspective_center_ver(center.y());

  } else if (proj == "Orthographic") {
    x.ci.ci_cam_type(CiCamType::orthographic);
    x.ci.ci_ortho_width(node.require("OrthoWidth").asFloat());
    x.ci.ci_ortho_width(node.require("OrthoHeight").asFloat());

  } else {
    throw runtime_error("Unknown projection type in metadata JSON file");
  }
  return x;
}

ViewParamsList::ViewParamsList(vector<ViewParams> viewParamsList)
    : vector<ViewParams>{move(viewParamsList)} {}

auto ViewParamsList::viewSizes() const -> SizeVector {
  SizeVector sizes;
  sizes.reserve(size());
  transform(begin(), end(), back_inserter(sizes),
            [](const ViewParams &viewParams) { return viewParams.ci.projectionPlaneSize(); });
  return sizes;
}

auto operator<<(ostream &stream, const ViewParamsList &viewParamsList) -> ostream & {
  for (size_t viewId = 0; viewId < viewParamsList.size(); ++viewId) {
    viewParamsList[viewId].printTo(stream, uint16_t(viewId));
  }
  return stream;
}

auto ViewParamsList::operator==(const ViewParamsList &other) const -> bool {
  return equal(begin(), end(), other.begin(), other.end());
}

auto ViewParamsList::loadFromJson(const Json &node, const vector<string> &names) -> ViewParamsList {
  ViewParamsList result;
  for (const auto &name : names) {
    for (size_t i = 0; i != node.size(); ++i) {
      if (name == node.at(i).require("Name").asString()) {
        result.push_back(ViewParams::loadFromJson(node.at(i)));
        break;
      }
    }
  }
  if (result.size() != names.size()) {
    throw runtime_error("Could not find all requested camera names in the metadata JSON file");
  }
  return result;
}
} // namespace TMIV::MivBitstream
