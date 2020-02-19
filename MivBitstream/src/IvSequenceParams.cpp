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

#include <TMIV/MivBitstream/IvSequenceParams.h>

#include "verify.h"
#include <TMIV/Common/Bitstream.h>
#include <TMIV/Common/Common.h>

#include <iomanip>
#include <iostream>

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto operator<<(ostream &stream, const ErpParams &projection) -> ostream & {
  return stream << "ERP " << projection.phiRange << " x " << projection.thetaRange << " deg";
}

auto operator<<(ostream &stream, const PerspectiveParams &projection) -> ostream & {
  return stream << "perspective " << projection.focal << ' ' << projection.center;
}

auto operator<<(ostream &stream, const ViewParams &viewParams) -> ostream & {
  if (!viewParams.name.empty()) {
    stream << "(" << setw(3) << viewParams.name << "), ";
  }
  stream << viewParams.size << ", ";
  visit([&](const auto &x) { stream << x; }, viewParams.projection);
  stream << ", norm. disp in [" << viewParams.dq.dq_norm_disp_low() << ", "
         << viewParams.dq.dq_norm_disp_high() << "] m^-1, hasOccupancy " << boolalpha
         << viewParams.hasOccupancy << ", depthOccMapThreshold "
         << viewParams.dq.dq_depth_occ_map_threshold_default();

  stream << format(", pose [%6.3f, %6.3f, %6.3f] m, [%6.3f, %6.3f, %6.3f] rad",
                   viewParams.ce.ce_view_pos_x(), viewParams.ce.ce_view_pos_y(),
                   viewParams.ce.ce_view_pos_z(), viewParams.ce.ce_view_quat_x(),
                   viewParams.ce.ce_view_quat_y(), viewParams.ce.ce_view_quat_z());

  if (viewParams.pruningChildren && !viewParams.pruningChildren->empty()) {
    stream << ", pruningChildren [ ";
    for (auto childId : *viewParams.pruningChildren) {
      stream << childId << " ";
    }
    stream << "]";
  }

  return stream;
}

auto ErpParams::operator==(const ErpParams &other) const -> bool {
  return phiRange == other.phiRange && thetaRange == other.thetaRange;
}

auto PerspectiveParams::operator==(const PerspectiveParams &other) const -> bool {
  return focal == other.focal && center == other.center;
}

auto ViewParams::operator==(const ViewParams &other) const -> bool {
  return size == other.size && ce == other.ce && projection == other.projection && dq == other.dq;
}

auto ViewParams::loadFromJson(const Json &node) -> ViewParams {
  ViewParams parameters;
  parameters.name = node.require("Name").asString();
  parameters.size = node.require("Resolution").asIntVector<2>();
  parameters.ce.position(node.require("Position").asFloatVector<3>());
  parameters.ce.eulerAngles(radperdeg * node.require("Rotation").asFloatVector<3>());
  const auto depthRange = node.require("Depth_range").asFloatVector<2>();
  constexpr auto kilometer = 1000.F;
  parameters.dq.dq_norm_disp_low(depthRange.y() < kilometer ? 1.F / depthRange.y() : 0.F);
  parameters.dq.dq_norm_disp_high(depthRange.x() < kilometer ? 1.F / depthRange.x() : 0.F);
  if (auto subnode = node.optional("HasInvalidDepth"); subnode) {
    parameters.hasOccupancy = subnode.asBool();
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
    if (front().dq != i->dq) {
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
    cameraParams.ce.ce_view_pos_x(bitstream.getFloat32());
    cameraParams.ce.ce_view_pos_y(bitstream.getFloat32());
    cameraParams.ce.ce_view_pos_z(bitstream.getFloat32());
    cameraParams.ce.ce_view_quat_x(bitstream.getFloat32());
    cameraParams.ce.ce_view_quat_y(bitstream.getFloat32());
    cameraParams.ce.ce_view_quat_z(bitstream.getFloat32());
  }

  const auto intrinsicParamsEqualFlag = bitstream.getFlag();

  for (auto viewParams = viewParamsList.begin(); viewParams != viewParamsList.end(); ++viewParams) {
    if (viewParams == viewParamsList.begin() || !intrinsicParamsEqualFlag) {
      auto camType = bitstream.getUint8();
      viewParams->size.x() = bitstream.getUint16() + 1;
      viewParams->size.y() = bitstream.getUint16() + 1;

      VERIFY_MIVBITSTREAM(camType < 2);
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
      VERIFY_MIVBITSTREAM(quantizationLaw == 0);
      viewParams->dq.dq_norm_disp_low(bitstream.getFloat32());
      viewParams->dq.dq_norm_disp_high(bitstream.getFloat32());
      viewParams->dq.dq_depth_occ_map_threshold_default(
          uint16_t(bitstream.readBits(depthOccMapThresholdNumBits)));
      viewParams->hasOccupancy = viewParams->dq.dq_depth_occ_map_threshold_default() > 0;
    } else {
      viewParams->dq = viewParamsList.front().dq;
      viewParams->hasOccupancy = viewParamsList.front().hasOccupancy;
    }
  }

  const auto pruningGraphParamsPresentFlag = bitstream.getFlag();

  if (pruningGraphParamsPresentFlag) {
    for (auto &viewParams : viewParamsList) {
      bool isLeaf = bitstream.getFlag();
      if (!isLeaf) {
        std::vector<std::uint16_t> childIdList(bitstream.getUVar(viewParamsList.size() - 1) + 1);

        for (auto &childId : childIdList) {
          childId = static_cast<std::uint16_t>(bitstream.getUVar(viewParamsList.size()));
        }
        viewParams.pruningChildren = std::move(childIdList);
      }
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
  VERIFY_MIVBITSTREAM(!empty() && size() - 1 <= UINT16_MAX);
  bitstream.putUint16(uint16_t(size() - 1));

  for (const auto &viewParams : *this) {
    bitstream.putFloat32(viewParams.ce.ce_view_pos_x());
    bitstream.putFloat32(viewParams.ce.ce_view_pos_y());
    bitstream.putFloat32(viewParams.ce.ce_view_pos_z());
    bitstream.putFloat32(viewParams.ce.ce_view_quat_x());
    bitstream.putFloat32(viewParams.ce.ce_view_quat_y());
    bitstream.putFloat32(viewParams.ce.ce_view_quat_z());
  }

  const auto intrinsicParamsEqualFlag = areIntrinsicParamsEqual();
  bitstream.putFlag(intrinsicParamsEqualFlag);

  for (const auto &viewParams : *this) {
    bitstream.putUint8(uint8_t(viewParams.projection.index()));
    VERIFY_MIVBITSTREAM(viewParams.size.x() >= 1 && viewParams.size.y() >= 1);
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
    bitstream.putFloat32(viewParams.dq.dq_norm_disp_low());
    bitstream.putFloat32(viewParams.dq.dq_norm_disp_high());
    bitstream.writeBits(viewParams.dq.dq_depth_occ_map_threshold_default(),
                        depthOccMapThresholdNumBits);

    if (depthQuantizationParamsEqualFlag) {
      break;
    }
  }

  bool pruningGraphParamsPresentFlag = std::any_of(begin(), end(), [](const auto &viewParams) {
    return (viewParams.pruningChildren && !viewParams.pruningChildren->empty());
  });

  bitstream.putFlag(pruningGraphParamsPresentFlag);

  if (pruningGraphParamsPresentFlag) {
    for (const auto &viewParams : *this) {
      if (viewParams.pruningChildren && !viewParams.pruningChildren->empty()) {

        bitstream.putFlag(false);

        const auto &childIdList = *viewParams.pruningChildren;
        bitstream.putUVar(childIdList.size() - 1, size() - 1);

        for (const auto &childId : childIdList) {
          bitstream.putUVar(childId, size());
        }
      } else {
        bitstream.putFlag(true);
      }
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

auto operator<<(std::ostream &stream, const IvSequenceParams &x) -> std::ostream & {
  stream << x.vps;
  stream << "view_params_list()=\n";
  stream << x.viewParamsList;

  if (x.viewingSpace) {
    stream << "viewing_space()=\n" << *x.viewingSpace;
  } else {
    stream << "No viewing space present\n";
  }

  return stream;
}

auto IvSequenceParams::operator==(const IvSequenceParams &other) const -> bool {
  return vps == other.vps && viewParamsList == other.viewParamsList &&
         viewingSpace == other.viewingSpace;
}

auto IvSequenceParams::decodeFrom(InputBitstream &bitstream) -> IvSequenceParams {
  auto x = IvSequenceParams{};

  x.viewParamsList = ViewParamsList::decodeFrom(bitstream, 10);

  x.msp().msp_depth_low_quality_flag(bitstream.getFlag());
  x.msp().msp_num_groups_minus1(unsigned(bitstream.getUExpGolomb()));
  x.msp().msp_max_entities_minus1(unsigned(bitstream.getUExpGolomb()));

  auto viewingSpace = optional<ViewingSpace>{};
  if (const auto viewingSpacePresentFlag = bitstream.getFlag(); viewingSpacePresentFlag) {
    viewingSpace = ViewingSpace::decodeFrom(bitstream);
  }

  const auto ivsSpExtensionPresentFlag = bitstream.getFlag();
  cout << "ivs_sp_extension_present_flag=" << boolalpha << ivsSpExtensionPresentFlag << '\n';

  return x;
}

void IvSequenceParams::encodeTo(OutputBitstream &bitstream) const {
  viewParamsList.encodeTo(bitstream, 10);
  bitstream.putFlag(msp().msp_depth_low_quality_flag());
  bitstream.putUExpGolomb(msp().msp_num_groups_minus1());
  bitstream.putUExpGolomb(msp().msp_max_entities_minus1());

  bitstream.putFlag(!!viewingSpace);
  if (viewingSpace) {
    viewingSpace->encodeTo(bitstream);
  }

  bitstream.putFlag(false);
}
} // namespace TMIV::MivBitstream
