/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#include <TMIV/Metadata/CameraParameterList.h>

#include <TMIV/Common/Json.h>

using namespace std;

namespace TMIV::Metadata {
CameraParameterList loadCamerasFromJson(const Common::Json &node,
                                        const std::vector<std::string> &names) {
  CameraParameterList result;
  for (const auto &name : names) {
    for (int i = 0; i != node.size(); ++i) {
      if (name == node.at(i).require("Name").asString()) {
        result.push_back(loadCameraFromJson(node.at(i)));
        break;
      }
    }
  }
  if (result.size() != names.size()) {
    throw runtime_error(
        "Could not find all requested camera names in the metadata JSON file");
  }
  return result;
}

CameraParameters loadCameraFromJson(uint16_t id, const Common::Json &node) {
  CameraParameters parameters;
  parameters.id = id;
  parameters.position = node.require("Position").asFloatVector<3>();
  parameters.rotation = node.require("Rotation").asFloatVector<3>();
  parameters.depthRange = node.require("Depth_range").asFloatVector<2>();

  auto proj = node.require("Projection").asString();
  if (proj == "Equirectangular") {
    parameters.type = ProjectionType::ERP;
    parameters.erpPhiRange = node.require("Rotation").asFloatVector<2>();
    parameters.erpThetaRange = node.require("Rotation").asFloatVector<2>();
  } else if (proj == "Perspective") {
    parameters.type = ProjectionType::Perspective;
    parameters.perspectiveFocal = node.require("Focal").asFloatVector<2>();
    parameters.perspectiveCenter =
        node.require("Principle_point").asFloatVector<2>();
  } else {
    throw runtime_error("Unknown projection type in metadata JSON file");
  }
  return parameters;
}
} // namespace TMIV::Metadata
