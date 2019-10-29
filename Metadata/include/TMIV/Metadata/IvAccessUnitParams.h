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

#ifndef _TMIV_METADATA_IVACCESSUNITPARAMS_H_
#define _TMIV_METADATA_IVACCESSUNITPARAMS_H_

#include <TMIV/Common/Vector.h>
#include <TMIV/Metadata/IvSequenceParams.h>

#include <cstdint>
#include <iosfwd>
#include <optional>
#include <vector>

namespace TMIV::Metadata {
enum class PatchRotation { none, swap, rot90, rot180, rot270, mirror, mrot90, mrot180 };

class InputBitstream;
class OutputBitstream;

// Data type that corresponds to the [ a ][ p ]-indiced entries of atlas_params( a ) and
// depth_occupancy( a, p ) for a single (a, p)-combination
struct AtlasParameters {
  // In specification: atlas_id[ i ] (= a)
  uint8_t atlasId{};

  // In specification: view_id[ a ][ p ]
  unsigned viewId{};

  // In specification: entity_id[ a ][ p ]
  std::optional<unsigned> entityId{};

  // In specification: patch_width_in_view_minus1[ a ][ p ]
  // In specification: patch_height_in_view_minus1[ a ][ p ]
  Common::Vec2i patchSizeInView;

  // In specification: patch_pos_in_view_x[ a ][ p ]
  // In specification: patch_pos_in_view_y[ a ][ p ]
  Common::Vec2i posInView;

  // In specification: patch_pos_in_atlas_x[ a ][ p ]
  // In specification: patch_pos_in_atlas_y[ a ][ p ]
  Common::Vec2i posInAtlas;

  // In specification: patch_rotation[ a ][ p ]
  PatchRotation rotation{};

  // In specification: depth_threshold_present_flag[ a ][ p ]
  // In specification: depth_occ_map_threshold[ a ][ p ]
  std::optional<uint16_t> depthOccMapThreshold;

  // In specification: depth_start_present_flag[ a ][ p ]
  // In specification: depth_start[ a ][ p ]
  std::optional<uint16_t> depthStart;

  // Is the patch rotated such that width and height swap?
  bool isRotated() const;

  // Return the size of the patch taking into account rotations
  auto patchSizeInAtlas() const -> Common::Vec2i;

  bool operator==(const AtlasParameters &other) const;
  bool operator!=(const AtlasParameters &other) const { return !operator==(other); };
};

// Data type that corresponds to the [ a ][ p ]-indiced fields of atlas_params( a ) and
// depth_occupancy( a, p ) over all (a, p) combinations. Note that p is not directly visible in this
// implementation but it is inferred to be element p counting only those for which atlasId == a.
using AtlasParamsVector = std::vector<AtlasParameters>;

// Data type that corresponds to atlas_params_list of specification
struct AtlasParamsList : public AtlasParamsVector {
  AtlasParamsList() = default;
  AtlasParamsList(AtlasParamsVector atlasParameters, bool omafV1CompatibleFlag_,
                  std::optional<std::vector<unsigned>> groupIds_, Common::SizeVector atlasSizes_,
                  std::vector<bool> depthOccupancyParamsPresentFlags_);
  AtlasParamsList(const AtlasParamsList &) = default;
  AtlasParamsList(AtlasParamsList &&) = default;
  AtlasParamsList &operator=(const AtlasParamsList &) = default;
  AtlasParamsList &operator=(AtlasParamsList &&) = default;

  // In specification: omaf_v1_compatible_flag
  bool omafV1CompatibleFlag{};

  // In specification: group_id[ i ] but stored as group_id[ a ]
  std::optional<std::vector<unsigned>> groupIds;

  // In specification: atlas_width_minus1[ a ]
  // In specification: atlas_height_minus1[ a ]
  Common::SizeVector atlasSizes;

  // In specification: depth_occ_params_present_flag[ a ]
  std::vector<bool> depthOccupancyParamsPresentFlags;

  void setAtlasParamsVector(AtlasParamsVector x) { AtlasParamsVector::operator=(move(x)); }

  friend std::ostream &operator<<(std::ostream &, const AtlasParamsList &);
  bool operator==(const AtlasParamsList &other) const;
  bool operator!=(const AtlasParamsList &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &, const IvSequenceParams &) -> AtlasParamsList;
  void encodeTo(OutputBitstream &, const IvSequenceParams &) const;
};

// Pixel position conversion from atlas to/from view
Common::Vec2i viewToAtlas(Common::Vec2i viewPosition, const AtlasParameters &patch);
Common::Vec2i atlasToView(Common::Vec2i atlasPosition, const AtlasParameters &patch);

struct IvAccessUnitParams {
  // In specification: atlas_params_list( )
  std::optional<AtlasParamsList> atlasParamsList;

  friend std::ostream &operator<<(std::ostream &, const IvAccessUnitParams &);
  bool operator==(const IvAccessUnitParams &other) const;
  bool operator!=(const IvAccessUnitParams &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &, const IvSequenceParams &) -> IvAccessUnitParams;
  void encodeTo(OutputBitstream &, const IvSequenceParams &) const;
};
} // namespace TMIV::Metadata

#endif
