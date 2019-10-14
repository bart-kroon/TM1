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

#include <cstdint>
#include <optional>
#include <vector>

namespace TMIV::Metadata {
enum class PatchRotation { none, swap, rot90, rot180, rot270, mirror, mrot90, mrot180 };

struct ViewParamsList;
class InputBitstream;
class OutputBitstream;

// Data type that corresponds to an entry of atlas_params of specification
struct AtlasParameters {
  // In specification: atlas_id
  uint8_t atlasId{};

  // In specification: view_id
  unsigned viewId{};

  // In specification: patch_{width,height}_in_view
  Common::Vec2i patchSizeInView;

  // In specification: patch_pos_in_view_{x,y}
  Common::Vec2i posInView;

  // In specification: patch_pos_in_atlas_{x,y}
  Common::Vec2i posInAtlas;

  // In specification: patch_rotation
  PatchRotation rotation{};

  // Is the patch rotated such that width and height swap?
  bool isRotated() const;

  // Return the size of the patch taking into account rotations
  auto patchSizeInAtlas() const -> Common::Vec2i;

  bool operator==(const AtlasParameters &other) const;
  bool operator!=(const AtlasParameters &other) const { return !operator==(other); };
};

using AtlasParametersVector = std::vector<AtlasParameters>;

// Data type that corresponds to atlas_params_list of specification
struct AtlasParamsList : public AtlasParametersVector {
  AtlasParamsList() = default;
  AtlasParamsList(AtlasParametersVector atlasParameters, bool omafV1CompatibleFlag_,
                  Common::SizeVector atlasSizes_)
      : AtlasParametersVector{std::move(atlasParameters)},
        omafV1CompatibleFlag{omafV1CompatibleFlag_}, atlasSizes{atlasSizes_} {}
  AtlasParamsList(const AtlasParamsList &) = default;
  AtlasParamsList(AtlasParamsList &&) = default;
  AtlasParamsList &operator=(const AtlasParamsList &) = default;
  AtlasParamsList &operator=(AtlasParamsList &&) = default;

  bool omafV1CompatibleFlag{};
  Common::SizeVector atlasSizes;

  static auto decodeFrom(InputBitstream &, const ViewParamsList &) -> AtlasParamsList;
  void encodeTo(OutputBitstream &, const ViewParamsList &) const;
};

// Pixel position conversion from atlas to/from view
Common::Vec2i viewToAtlas(Common::Vec2i viewPosition, const AtlasParameters &patch);
Common::Vec2i atlasToView(Common::Vec2i atlasPosition, const AtlasParameters &patch);

struct IvAccessUnitParams {
  std::optional<AtlasParamsList> atlasParamsList;

  bool operator==(const IvAccessUnitParams &other) const;
  bool operator!=(const IvAccessUnitParams &other) const { return !operator==(other); }

  static auto decodeFrom(InputBitstream &, const ViewParamsList &cameras) -> IvAccessUnitParams;
  void encodeTo(OutputBitstream &, const ViewParamsList &cameras) const;
};
} // namespace TMIV::Metadata

#endif
