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

#ifndef _TMIV_ATLASDECONSTRUCTOR_ATLASDECONSTRUCTOR_H_
#define _TMIV_ATLASDECONSTRUCTOR_ATLASDECONSTRUCTOR_H_

#include <TMIV/AtlasDeconstructor/IAtlasDeconstructor.h>

#include <TMIV/Common/Json.h>

namespace TMIV::AtlasDeconstructor {
class AtlasDeconstructor : public IAtlasDeconstructor {
public:
  AtlasDeconstructor(const Common::Json & /*unused*/, const Common::Json & /*unused*/);
  AtlasDeconstructor(const AtlasDeconstructor &) = delete;
  AtlasDeconstructor(AtlasDeconstructor &&) = default;
  AtlasDeconstructor &operator=(const AtlasDeconstructor &) = delete;
  AtlasDeconstructor &operator=(AtlasDeconstructor &&) = default;
  ~AtlasDeconstructor() override = default;

  auto getPatchIdMap(const Common::SizeVector &atlasSize,
                     const Metadata::AtlasParametersVector &atlasParamsVector,
                     const Metadata::ViewParamsVector &viewParamsVector,
                     const Common::MVD10Frame &frame) -> Common::PatchIdMapList override;
  auto recoverPrunedView(const Common::MVD10Frame &atlas,
                         const Metadata::ViewParamsVector &viewParamsVector,
                         const Metadata::AtlasParametersVector &atlasParamsVector)
      -> Common::MVD10Frame override;

private:
  void writePatchIdInMap(const Metadata::AtlasParameters &patch,
                         Common::PatchIdMapList &patchMapList, std::uint16_t patchId,
                         const Common::MVD10Frame &frame, std::uint16_t depthOccMapThreshold) const;
};
} // namespace TMIV::AtlasDeconstructor

#endif
