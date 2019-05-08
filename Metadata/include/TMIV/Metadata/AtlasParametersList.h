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

#ifndef _TMIV_METADATA_ATLASPARAMETERSLIST_H_
#define _TMIV_METADATA_ATLASPARAMETERSLIST_H_

#include <cstdint>
#include <string>
#include <vector>

#include <TMIV/Common/Vector.h>

namespace TMIV::Metadata {
using Vec2i = TMIV::Common::Vec2i;

enum class PatchRotation {
  upright, // what was up stays up
  ccw      // what was up goes left
};

// Data type that corresponds to an entry of atlas_params of MPEG/N18464
struct AtlasParameters {
  // In MPEG/N18464: atlas_id
  uint8_t atlasId{};

  // In MPEG/N18464: view_id
  uint8_t viewId{};

  // In MPEG/N18464: patch_{width,height}_in_view
  Vec2i patchSize;

  // In MPEG/N18464: patch_pos_in_view_{x,y}
  Vec2i posInView;

  // In MPEG/N18464: patch_pos_in_atlas_{x,y}
  Vec2i posInAtlas;

  // In MPEG/N18464: patch_rotation
  PatchRotation rotation;
};

static_assert(sizeof(AtlasParameters) == 32);

std::string PatchParametersString(const AtlasParameters &patchParameters);

// Data type that corresponds to atlas_params_list of MPEG/N18464
using AtlasParametersList = std::vector<AtlasParameters>;
} // namespace TMIV::Metadata

#endif
