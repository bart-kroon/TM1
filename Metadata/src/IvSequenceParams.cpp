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

#include <TMIV/Metadata/IvSequenceParams.h>

#include "verify.h"
#include <TMIV/Common/Common.h>
#include <TMIV/Metadata/Bitstream.h>

#include <cassert>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::Metadata {
bool IvsProfileTierLevel::operator==(const IvsProfileTierLevel & /* unused */) const {
  return true;
}

auto IvsProfileTierLevel::decodeFrom(InputBitstream & /* unused */) -> IvsProfileTierLevel {
  return {};
}

void IvsProfileTierLevel::encodeTo(OutputBitstream & /* unused */) const {}

ostream &operator<<(ostream &stream, const ErpParams &projection) {
  return stream << "ERP " << projection.phiRange << " x " << projection.thetaRange << " deg";
}

ostream &operator<<(ostream &stream, const PerspectiveParams &projection) {
  return stream << "perspective " << projection.focal << ' ' << projection.center;
}

ostream &operator<<(ostream &stream, const ViewParams &camera) {
  stream << camera.size << ", ";
  visit([&](const auto &x) { stream << x; }, camera.projection);
  stream << ", norm. disp in " << camera.normDispRange << " m^-1, depthOccMapThreshold "
         << camera.depthOccMapThreshold << ", pose "
         << format("[%6.3f, %6.3f, %6.3f] m, ", camera.position.x(), camera.position.y(),
                   camera.position.z())
         << camera.rotation << " deg";
  return stream;
}

bool ErpParams::operator==(const ErpParams &other) const {
  return phiRange == other.phiRange && thetaRange == other.thetaRange;
}

bool PerspectiveParams::operator==(const PerspectiveParams &other) const {
  return focal == other.focal && center == other.center;
}

bool ViewParams::operator==(const ViewParams &other) const {
  return size == other.size && position == other.position && rotation == other.rotation &&
         projection == other.projection && normDispRange == other.normDispRange &&
         depthOccMapThreshold == other.depthOccMapThreshold;
}

ViewParams ViewParams::loadFromJson(const Json &node) {
  ViewParams parameters;
  parameters.size = node.require("Resolution").asIntVector<2>();
  parameters.position = node.require("Position").asFloatVector<3>();
  parameters.rotation = node.require("Rotation").asFloatVector<3>();
  const auto depthRange = node.require("Depth_range").asFloatVector<2>();
  constexpr auto kilometer = 1000.F;
  parameters.normDispRange.x() = depthRange.y() < kilometer ? 1.F / depthRange.y() : 0.F;
  parameters.normDispRange.y() = depthRange.x() < kilometer ? 1.F / depthRange.x() : 0.F;
  if (auto subnode = node.optional("HasInvalidDepth"); subnode) {
    parameters.depthOccMapThreshold = subnode.asBool() ? 1 : 0;
  }

  auto proj = node.require("Projection").asString();
  if (proj == "Equirectangular") {
    parameters.projection = ErpParams{node.require("Hor_range").asFloatVector<2>(),
                                      node.require("Ver_range").asFloatVector<2>()};
  } else if (proj == "Perspective") {
    parameters.projection = PerspectiveParams{node.require("Focal").asFloatVector<2>(),
                                              node.require("Principle_point").asFloatVector<2>()};
  } else {
    throw runtime_error("Unknown projection type in metadata JSON file");
  }
  return parameters;
}

auto modifyDepthRange(const CameraParametersVector &in) -> CameraParametersVector {
  auto out = CameraParametersVector{};
  out.reserve(in.size());
  transform(begin(in), end(in), back_inserter(out), [](ViewParams x) {
    if (x.depthOccMapThreshold == 0) {
      return x;
    }
    x.depthOccMapThreshold = 64; // =T
    const auto nearLevel = 1023.F;
    const auto farLevel = float(2 * x.depthOccMapThreshold);
    // Mapping is [2T, 1023] --> [old far, near]. What is level 0? (the new far)
    x.normDispRange[0] +=
        (0.F - farLevel) / (nearLevel - farLevel) * (x.normDispRange[1] - x.normDispRange[0]);
    return x;
  });
  return out;
}

bool CameraParamsList::areIntrinsicParamsEqual() const {
  for (auto i = begin() + 1; i < end(); ++i) {
    if (front().projection != i->projection) {
      return false;
    }
  }

  return true;
}

bool CameraParamsList::areDepthQuantizationParamsEqual() const {
  for (auto i = begin() + 1; i < end(); ++i) {
    if (front().normDispRange != i->normDispRange) {
      return false;
    }
  }

  return true;
}

ostream &operator<<(ostream &stream, const CameraParamsList &cameras) {
  for (size_t i = 0; i < cameras.size(); ++i) {
    stream << "Camera " << setw(2) << i << ": " << cameras[i] << '\n';
  }
  return stream;
}

bool CameraParamsList::operator==(const CameraParamsList &other) const {
  return equal(begin(), end(), other.begin(), other.end());
}

auto ErpParams::decodeFrom(InputBitstream &bitstream) -> ErpParams {
  ErpParams projection;
  projection.phiRange.x() = bitstream.getFloat32();
  projection.phiRange.y() = bitstream.getFloat32();
  projection.thetaRange.x() = bitstream.getFloat32();
  projection.thetaRange.y() = bitstream.getFloat32();
  return projection;
}

auto PerspectiveParams::decodeFrom(InputBitstream &bitstream) -> PerspectiveParams {
  PerspectiveParams projection;
  projection.focal.x() = bitstream.getFloat32();
  projection.focal.y() = bitstream.getFloat32();
  projection.center.x() = bitstream.getFloat32();
  projection.center.y() = bitstream.getFloat32();
  return projection;
}

auto CameraParamsList::decodeFrom(InputBitstream &bitstream) -> CameraParamsList {
  auto cameraParamsList = CameraParamsList{CameraParametersVector(bitstream.getUint16() + 1)};

  for (auto &cameraParams : cameraParamsList) {
    cameraParams.position.x() = bitstream.getFloat32();
    cameraParams.position.y() = bitstream.getFloat32();
    cameraParams.position.z() = bitstream.getFloat32();
    cameraParams.rotation.x() = bitstream.getFloat32();
    cameraParams.rotation.y() = bitstream.getFloat32();
    cameraParams.rotation.z() = bitstream.getFloat32();
  }

  const auto intrinsicParamsEqualFlag = bitstream.getFlag();

  for (auto camera = cameraParamsList.begin(); camera != cameraParamsList.end(); ++camera) {
    if (camera == cameraParamsList.begin() || !intrinsicParamsEqualFlag) {
      auto camType = bitstream.getUint8();
      camera->size.x() = bitstream.getUint16();
      camera->size.y() = bitstream.getUint16();

      verify(camType < 2);
      switch (camType) {
      case 0:
        camera->projection = ErpParams::decodeFrom(bitstream);
        break;
      case 1:
        camera->projection = PerspectiveParams::decodeFrom(bitstream);
        break;
      default:
        abort();
      }
    } else {
      camera->size = cameraParamsList.front().size;
      camera->projection = cameraParamsList.front().projection;
    }
  }

  const auto depthQuantizationParamsEqualFlag = bitstream.getFlag();

  for (auto camera = cameraParamsList.begin(); camera != cameraParamsList.end(); ++camera) {
    if (camera == cameraParamsList.begin() || !depthQuantizationParamsEqualFlag) {
      const auto quantizationLaw = bitstream.getUint8();
      verify(quantizationLaw == 0);
      camera->normDispRange.x() = bitstream.getFloat32();
      camera->normDispRange.y() = bitstream.getFloat32();
      camera->depthOccMapThreshold = bitstream.getUint16();
    } else {
      camera->normDispRange = cameraParamsList.front().normDispRange;
      camera->depthOccMapThreshold = cameraParamsList.front().depthOccMapThreshold;
    }
  }

  return cameraParamsList;
}

void ErpParams::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putFloat32(phiRange.x());
  bitstream.putFloat32(phiRange.y());
  bitstream.putFloat32(thetaRange.x());
  bitstream.putFloat32(thetaRange.y());
}

void PerspectiveParams::encodeTo(OutputBitstream &bitstream) const {
  bitstream.putFloat32(focal.x());
  bitstream.putFloat32(focal.y());
  bitstream.putFloat32(center.x());
  bitstream.putFloat32(center.y());
}

void CameraParamsList::encodeTo(OutputBitstream &bitstream) const {
  assert(!empty() && size() - 1 <= UINT16_MAX);
  bitstream.putUint16(uint16_t(size() - 1));

  for (const auto &camera : *this) {
    bitstream.putFloat32(camera.position.x());
    bitstream.putFloat32(camera.position.y());
    bitstream.putFloat32(camera.position.z());
    bitstream.putFloat32(camera.rotation.x());
    bitstream.putFloat32(camera.rotation.y());
    bitstream.putFloat32(camera.rotation.z());
  }

  const auto intrinsicParamsEqualFlag = areIntrinsicParamsEqual();
  bitstream.putFlag(intrinsicParamsEqualFlag);

  for (const auto &camera : *this) {
    bitstream.putUint8(uint8_t(camera.projection.index()));
    bitstream.putUint16(uint16_t(camera.size.x()));
    bitstream.putUint16(uint16_t(camera.size.y()));
    visit([&](const auto &x) { x.encodeTo(bitstream); }, camera.projection);
    if (intrinsicParamsEqualFlag) {
      break;
    }
  }

  const auto depthQuantizationParamsEqualFlag = areDepthQuantizationParamsEqual();
  bitstream.putFlag(depthQuantizationParamsEqualFlag);

  for (const auto &camera : *this) {
    bitstream.putUint8(0); // quantization_law
    bitstream.putFloat32(camera.normDispRange.x());
    bitstream.putFloat32(camera.normDispRange.y());
    bitstream.putUint16(camera.depthOccMapThreshold);

    if (depthQuantizationParamsEqualFlag) {
      break;
    }
  }
}

CameraParamsList CameraParamsList::loadFromJson(const Json &node, const vector<string> &names) {
  CameraParamsList result;
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

bool IvSequenceParams::operator==(const IvSequenceParams &other) const {
  return ivsProfileTierLevel == other.ivsProfileTierLevel &&
         cameraParamsList == other.cameraParamsList;
}

auto IvSequenceParams::decodeFrom(InputBitstream &bitstream) -> IvSequenceParams {
  const auto ivsProfileTierLevel = IvsProfileTierLevel::decodeFrom(bitstream);
  const auto cameraParamsList = CameraParamsList::decodeFrom(bitstream);
  const auto ivsSpExtensionPresentFlag = bitstream.getFlag();
  cout << "ivs_sp_extension_data_flag=" << boolalpha << ivsSpExtensionPresentFlag << '\n';
  return IvSequenceParams{ivsProfileTierLevel, cameraParamsList};
}

void IvSequenceParams::encodeTo(OutputBitstream &bitstream) const {
  ivsProfileTierLevel.encodeTo(bitstream);
  cameraParamsList.encodeTo(bitstream);
  bitstream.putFlag(false);
}
} // namespace TMIV::Metadata
