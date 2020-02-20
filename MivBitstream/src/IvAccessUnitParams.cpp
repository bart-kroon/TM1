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

#include <TMIV/MivBitstream/IvAccessUnitParams.h>

#include <TMIV/Common/Bitstream.h>

#include <algorithm>
#include <iostream>
#include <map>

#include "verify.h"

using namespace std;
using namespace TMIV::Common;

namespace TMIV::MivBitstream {
auto PatchParams::isRotated() const -> bool {
  return rotation == FlexiblePatchOrientation::FPO_ROT90 ||
         rotation == FlexiblePatchOrientation::FPO_SWAP ||
         rotation == FlexiblePatchOrientation::FPO_ROT270 ||
         rotation == FlexiblePatchOrientation::FPO_MROT90;
}

auto PatchParams::patchSizeInAtlas() const -> Vec2i {
  if (isRotated()) {
    return {patchSizeInView.y(), patchSizeInView.x()};
  }
  return patchSizeInView;
}

auto PatchParams::operator==(const PatchParams &other) const -> bool {
  return vuhAtlasId == other.vuhAtlasId && pduViewId == other.pduViewId && pduEntityId == other.pduEntityId &&
         patchSizeInView == other.patchSizeInView && posInView == other.posInView &&
         posInAtlas == other.posInAtlas && rotation == other.rotation &&
         pduDepthOccMapThreshold == other.pduDepthOccMapThreshold && pduDepthStart == other.pduDepthStart;
}

auto AtlasParamsList::operator==(const AtlasParamsList &other) const -> bool {
  return equal(begin(), end(), other.begin(), other.end()) &&
         omafV1CompatibleFlag == other.omafV1CompatibleFlag && groupIds == other.groupIds &&
         atlasSizes == other.atlasSizes &&
         depthOccupancyParamsPresentFlags == other.depthOccupancyParamsPresentFlags;
}

auto operator<<(ostream &stream, const AtlasParamsList &atlasParamsList) -> ostream & {
  stream << "num_patches=" << atlasParamsList.size() << '\n';
  stream << "omaf_v1_compatible_flag=" << boolalpha << atlasParamsList.omafV1CompatibleFlag << '\n';

  if (atlasParamsList.groupIds) {
    stream << "groupIds={";
    auto sep = "";
    for (auto &groupId : *atlasParamsList.groupIds) {
      stream << sep << groupId;
      sep = ", ";
    }
    stream << "}\n";
  } else {
    stream << "No group ID's\n";
  }

  stream << "atlas_size[]={";
  auto sep = "";
  for (auto &atlasSize : atlasParamsList.atlasSizes) {
    stream << sep << atlasSize;
    sep = ", ";
  }
  stream << "}\n";

  stream << "depth_occ_params_present_flag[]={";
  sep = "";
  for (auto depthOccupancyParamsPresentFlag : atlasParamsList.depthOccupancyParamsPresentFlags) {
    stream << sep << depthOccupancyParamsPresentFlag;
    sep = ", ";
  }
  stream << "}\n";

  return stream << '\n';
}

namespace {
// Use vector<T> as a map<size_t, T>
template <typename Vector, typename Value>
void assignAt(Vector &vector, size_t position, Value &&value) {
  while (vector.size() <= position) {
    vector.emplace_back();
  }
  vector[position] = forward<Value>(value);
}
} // namespace

auto viewToAtlas(Vec2i viewPosition, const PatchParams &patch) -> Vec2i {
  int w = patch.patchSizeInView.x();
  int h = patch.patchSizeInView.y();
  int xM = patch.posInView.x();
  int yM = patch.posInView.y();
  int xP = patch.posInAtlas.x();
  int yP = patch.posInAtlas.y();
  int x = viewPosition.x();
  int y = viewPosition.y();

  switch (patch.rotation) {
  case FlexiblePatchOrientation::FPO_NULL: // (x, y)
    return {x - xM + xP, y - yM + yP};
  case FlexiblePatchOrientation::FPO_SWAP: // (y, x)
    return {y - yM + xP, x - xM + yP};
  case FlexiblePatchOrientation::FPO_ROT90: // (-y, x)
    return {-y + yM + xP + h - 1, x - xM + yP};
  case FlexiblePatchOrientation::FPO_ROT180: // (-x, -y)
    return {-x + xM + xP + w - 1, -y + yM + yP + h - 1};
  case FlexiblePatchOrientation::FPO_ROT270: // (y, -x)
    return {y - yM + xP, -x + xM + yP + w - 1};
  case FlexiblePatchOrientation::FPO_MIRROR: // (-x, y)
    return {-x + xM + xP + w - 1, y - yM + yP};
  case FlexiblePatchOrientation::FPO_MROT90: // (-y, -x)
    return {-y + yM + xP + h - 1, -x + xM + yP + w - 1};
  case FlexiblePatchOrientation::FPO_MROT180: // (x, -y)
    return {x - xM + xP, -y + yM + yP + h - 1};
  default:
    abort();
  }
}

auto atlasToView(Vec2i atlasPosition, const PatchParams &patch) -> Vec2i {
  int w = patch.patchSizeInView.x();
  int h = patch.patchSizeInView.y();
  int xM = patch.posInView.x();
  int yM = patch.posInView.y();
  int xP = patch.posInAtlas.x();
  int yP = patch.posInAtlas.y();
  int x = atlasPosition.x();
  int y = atlasPosition.y();

  switch (patch.rotation) {
  case FlexiblePatchOrientation::FPO_NULL: // (x, y)
    return {x - xP + xM, y - yP + yM};
  case FlexiblePatchOrientation::FPO_SWAP: // (y, x)
    return {y - yP + xM, x - xP + yM};
  case FlexiblePatchOrientation::FPO_ROT90: // (y, -x)
    return {y - yP + xM, -x + xP + yM + h - 1};
  case FlexiblePatchOrientation::FPO_ROT180: // (-x, -y)
    return {-x + xP + xM + w - 1, -y + yP + yM + h - 1};
  case FlexiblePatchOrientation::FPO_ROT270: // (-y, x)
    return {-y + yP + xM + w - 1, x - xP + yM};
  case FlexiblePatchOrientation::FPO_MIRROR: // (-x, y)
    return {-x + xP + xM + w - 1, y - yP + yM};
  case FlexiblePatchOrientation::FPO_MROT90: // (-y, -x)
    return {-y + yP + xM + w - 1, -x + xP + yM + h - 1};
  case FlexiblePatchOrientation::FPO_MROT180: // (x, -y)
    return {x - xP + xM, -y + yP + yM + h - 1};
  default:
    abort();
  }
}

auto operator<<(ostream &stream, const IvAccessUnitParams &ivAccessUnitParams) -> ostream & {
  return stream << ivAccessUnitParams.atlasParamsList;
}

auto IvAccessUnitParams::operator==(const IvAccessUnitParams &other) const -> bool {
  return atlasParamsList == other.atlasParamsList;
}
} // namespace TMIV::MivBitstream
