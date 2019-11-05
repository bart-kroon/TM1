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
auto operator<<(std::ostream &stream, const IvsProfileTierLevel & /* unused */) -> std::ostream & {
  return stream << "{}";
}

auto IvsProfileTierLevel::operator==(const IvsProfileTierLevel & /* unused */) const -> bool {
  return true;
}

auto IvsProfileTierLevel::decodeFrom(InputBitstream & /* unused */) -> IvsProfileTierLevel {
  return {};
}

void IvsProfileTierLevel::encodeTo(OutputBitstream & /* unused */) const {}

auto operator<<(ostream &stream, const ErpParams &projection) -> ostream & {
  return stream << "ERP " << projection.phiRange << " x " << projection.thetaRange << " deg";
}

auto operator<<(ostream &stream, const PerspectiveParams &projection) -> ostream & {
  return stream << "perspective " << projection.focal << ' ' << projection.center;
}

auto operator<<(ostream &stream, const ViewParams &viewParams) -> ostream & {
  stream << "(" << viewParams.name << "), " << viewParams.size << ", ";
  visit([&](const auto &x) { stream << x; }, viewParams.projection);
  stream << ", norm. disp in " << viewParams.normDispRange << " m^-1, depthOccMapThreshold "
         << viewParams.depthOccMapThreshold;

  if (viewParams.depthStart) {
    stream << ", depthStart " << *viewParams.depthStart;
  }

  stream << ", pose "
         << format("[%6.3f, %6.3f, %6.3f] m, ", viewParams.position.x(), viewParams.position.y(),
                   viewParams.position.z())
         << viewParams.rotation << " deg";
  return stream;
}

auto ErpParams::operator==(const ErpParams &other) const -> bool {
  return phiRange == other.phiRange && thetaRange == other.thetaRange;
}

auto PerspectiveParams::operator==(const PerspectiveParams &other) const -> bool {
  return focal == other.focal && center == other.center;
}

auto ViewParams::operator==(const ViewParams &other) const -> bool {
  return size == other.size && position == other.position && rotation == other.rotation &&
         projection == other.projection && normDispRange == other.normDispRange &&
         depthOccMapThreshold == other.depthOccMapThreshold && depthStart == other.depthStart;
}

auto ViewParams::loadFromJson(const Json &node) -> ViewParams {
  ViewParams parameters;
  parameters.name = node.require("Name").asString();
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

auto ViewParamsList::areIntrinsicParamsEqual() const -> bool {
  for (auto i = begin() + 1; i < end(); ++i) {
    if (front().projection != i->projection) {
      return false;
    }
  }

  return true;
}

auto ViewParamsList::areDepthQuantizationParamsEqual() const -> bool {
  for (auto i = begin() + 1; i < end(); ++i) {
    if (front().normDispRange != i->normDispRange) {
      return false;
    }
  }

  return true;
}

auto ViewParamsList::viewSizes() const -> SizeVector {
  SizeVector sizes;
  sizes.reserve(size());
  transform(begin(), end(), back_inserter(sizes),
            [](const ViewParams &viewParams) { return viewParams.size; });
  return sizes;
}

auto operator<<(ostream &stream, const ViewParamsList &viewParamsVector) -> ostream & {
  for (size_t i = 0; i < viewParamsVector.size(); ++i) {
    stream << "View " << setw(2) << i << ": " << viewParamsVector[i] << '\n';
  }
  return stream;
}

auto ViewParamsList::operator==(const ViewParamsList &other) const -> bool {
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

auto ViewParamsList::decodeFrom(InputBitstream &bitstream, unsigned depthOccMapThresholdNumBits)
    -> ViewParamsList {
  auto viewParamsList = ViewParamsList{ViewParamsVector(bitstream.getUint16() + 1)};

  for (auto &cameraParams : viewParamsList) {
    cameraParams.position.x() = bitstream.getFloat32();
    cameraParams.position.y() = bitstream.getFloat32();
    cameraParams.position.z() = bitstream.getFloat32();
    cameraParams.rotation.x() = bitstream.getFloat32();
    cameraParams.rotation.y() = bitstream.getFloat32();
    cameraParams.rotation.z() = bitstream.getFloat32();
  }

  const auto intrinsicParamsEqualFlag = bitstream.getFlag();

  for (auto viewParams = viewParamsList.begin(); viewParams != viewParamsList.end(); ++viewParams) {
    if (viewParams == viewParamsList.begin() || !intrinsicParamsEqualFlag) {
      auto camType = bitstream.getUint8();
      viewParams->size.x() = bitstream.getUint16() + 1;
      viewParams->size.y() = bitstream.getUint16() + 1;

      verify(camType < 2);
      switch (camType) {
      case 0:
        viewParams->projection = ErpParams::decodeFrom(bitstream);
        break;
      case 1:
        viewParams->projection = PerspectiveParams::decodeFrom(bitstream);
        break;
      default:
        abort();
      }
    } else {
      viewParams->size = viewParamsList.front().size;
      viewParams->projection = viewParamsList.front().projection;
    }
  }

  const auto depthQuantizationParamsEqualFlag = bitstream.getFlag();

  for (auto viewParams = viewParamsList.begin(); viewParams != viewParamsList.end(); ++viewParams) {
    if (viewParams == viewParamsList.begin() || !depthQuantizationParamsEqualFlag) {
      const auto quantizationLaw = bitstream.getUint8();
      verify(quantizationLaw == 0);
      viewParams->normDispRange.x() = bitstream.getFloat32();
      viewParams->normDispRange.y() = bitstream.getFloat32();
      viewParams->depthOccMapThreshold = uint16_t(bitstream.readBits(depthOccMapThresholdNumBits));

      if (const auto depthStartDefaultPresentFlag = bitstream.getFlag();
          depthStartDefaultPresentFlag) {
        viewParams->depthStart = uint16_t(bitstream.readBits(depthOccMapThresholdNumBits));
      }
    } else {
      viewParams->normDispRange = viewParamsList.front().normDispRange;
      viewParams->depthOccMapThreshold = viewParamsList.front().depthOccMapThreshold;
    }
  }

  return viewParamsList;
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

void ViewParamsList::encodeTo(OutputBitstream &bitstream,
                              unsigned depthOccMapThresholdNumBits) const {
  assert(!empty() && size() - 1 <= UINT16_MAX);
  bitstream.putUint16(uint16_t(size() - 1));

  for (const auto &viewParams : *this) {
    bitstream.putFloat32(viewParams.position.x());
    bitstream.putFloat32(viewParams.position.y());
    bitstream.putFloat32(viewParams.position.z());
    bitstream.putFloat32(viewParams.rotation.x());
    bitstream.putFloat32(viewParams.rotation.y());
    bitstream.putFloat32(viewParams.rotation.z());
  }

  const auto intrinsicParamsEqualFlag = areIntrinsicParamsEqual();
  bitstream.putFlag(intrinsicParamsEqualFlag);

  for (const auto &viewParams : *this) {
    bitstream.putUint8(uint8_t(viewParams.projection.index()));
    assert(viewParams.size.x() >= 1 && viewParams.size.y() >= 1);
    bitstream.putUint16(uint16_t(viewParams.size.x() - 1));
    bitstream.putUint16(uint16_t(viewParams.size.y() - 1));
    visit([&](const auto &x) { x.encodeTo(bitstream); }, viewParams.projection);
    if (intrinsicParamsEqualFlag) {
      break;
    }
  }

  const auto depthQuantizationParamsEqualFlag = areDepthQuantizationParamsEqual();
  bitstream.putFlag(depthQuantizationParamsEqualFlag);

  for (const auto &viewParams : *this) {
    bitstream.putUint8(0); // quantization_law
    bitstream.putFloat32(viewParams.normDispRange.x());
    bitstream.putFloat32(viewParams.normDispRange.y());
    bitstream.writeBits(viewParams.depthOccMapThreshold, depthOccMapThresholdNumBits);

    bitstream.putFlag(!!viewParams.depthStart);
    if (viewParams.depthStart) {
      bitstream.writeBits(*viewParams.depthStart, depthOccMapThresholdNumBits);
    }

    if (depthQuantizationParamsEqualFlag) {
      break;
    }
  }
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

auto operator<<(std::ostream &stream, const IvSequenceParams &ivSequenceParams) -> std::ostream & {
  stream << "ivs_profile_tier_level()=" << ivSequenceParams.ivsProfileTierLevel << '\n';
  stream << "depth_low_quality_flag=" << boolalpha << ivSequenceParams.depthLowQualityFlag << '\n';
  stream << "num_groups=" << ivSequenceParams.numGroups << '\n';
  stream << "max_entities=" << ivSequenceParams.maxEntities << '\n';
  stream << "depth_occ_map_threshold_num_bits=" << ivSequenceParams.depthOccMapThresholdNumBits
         << '\n';
  stream << "view_params_list()=\n";
  stream << ivSequenceParams.viewParamsList;

  if (ivSequenceParams.viewingSpace) {
    stream << "viewing_space()=\n" << *ivSequenceParams.viewingSpace;
  } else {
    stream << "No viewing space present\n";
  }

  return stream;
}

auto IvSequenceParams::operator==(const IvSequenceParams &other) const -> bool {
  return ivsProfileTierLevel == other.ivsProfileTierLevel &&
         viewParamsList == other.viewParamsList &&
         depthLowQualityFlag == other.depthLowQualityFlag && numGroups == other.numGroups &&
         maxEntities == other.maxEntities &&
         depthOccMapThresholdNumBits == other.depthOccMapThresholdNumBits &&
         viewingSpace == other.viewingSpace;
}

auto IvSequenceParams::decodeFrom(InputBitstream &bitstream) -> IvSequenceParams {
  const auto ivsProfileTierLevel = IvsProfileTierLevel::decodeFrom(bitstream);
  const auto depthOccMapThresholdNumBits = unsigned(8 + bitstream.readBits(4));
  const auto viewParamsList = ViewParamsList::decodeFrom(bitstream, depthOccMapThresholdNumBits);
  const auto depthLowQualityFlag = bitstream.getFlag();
  const auto numGroups = unsigned(1 + bitstream.getUExpGolomb());
  const auto maxEntities = unsigned(1 + bitstream.getUExpGolomb());

  auto viewingSpace = optional<ViewingSpace>{};
  if (const auto viewingSpacePresentFlag = bitstream.getFlag(); viewingSpacePresentFlag) {
    viewingSpace = ViewingSpace::decodeFrom(bitstream);
  }

  const auto ivsSpExtensionPresentFlag = bitstream.getFlag();
  cout << "ivs_sp_extension_present_flag=" << boolalpha << ivsSpExtensionPresentFlag << '\n';
  return IvSequenceParams{ivsProfileTierLevel, viewParamsList, depthLowQualityFlag,
                          numGroups,           maxEntities,    depthOccMapThresholdNumBits,
                          viewingSpace};
}

void IvSequenceParams::encodeTo(OutputBitstream &bitstream) const {
  ivsProfileTierLevel.encodeTo(bitstream);
  bitstream.writeBits(depthOccMapThresholdNumBits - 8, 4);
  viewParamsList.encodeTo(bitstream, depthOccMapThresholdNumBits);
  bitstream.putFlag(depthLowQualityFlag);
  verify(numGroups >= 1);
  bitstream.putUExpGolomb(numGroups - 1);
  verify(maxEntities >= 1);
  bitstream.putUExpGolomb(maxEntities - 1);
  verify(depthOccMapThresholdNumBits >= 8);

  bitstream.putFlag(!!viewingSpace);
  if (viewingSpace) {
    viewingSpace->encodeTo(bitstream);
  }

  bitstream.putFlag(false);
}
} // namespace TMIV::Metadata
