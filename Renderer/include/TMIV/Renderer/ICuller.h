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

#ifndef _TMIV_RENDERER_ICULLER_H_
#define _TMIV_RENDERER_ICULLER_H_

#include <TMIV/Common/Frame.h>
#include <TMIV/Metadata/IvAccessUnitParams.h>
#include <TMIV/Metadata/IvSequenceParams.h>

namespace TMIV::Renderer {
class ICuller {
public:
  ICuller() = default;
  ICuller(const ICuller &) = delete;
  ICuller(ICuller &&) = default;
  ICuller &operator=(const ICuller &) = delete;
  ICuller &operator=(ICuller &&) = default;
  virtual ~ICuller() = default;

  // Do sub-block culling and update the PatchIdMap
  virtual auto updatePatchIdmap(const Common::MVD10Frame &atlas, const Common::PatchIdMapList &maps,
                                const Metadata::IvSequenceParams &ivSequenceParams,
                                const Metadata::IvAccessUnitParams &ivAccessUnitParams,
                                const Metadata::ViewParams &target) -> Common::PatchIdMapList = 0;
};

class NoCuller : public ICuller {
public:
  NoCuller(const Common::Json & /*unused*/, const Common::Json & /*unused*/) {}
  NoCuller(const NoCuller &) = delete;
  NoCuller(NoCuller &&) = default;
  NoCuller &operator=(const NoCuller &) = delete;
  NoCuller &operator=(NoCuller &&) = default;
  ~NoCuller() override = default;

  // Do sub-block culling and update the PatchIdMap
  auto updatePatchIdmap(const Common::MVD10Frame &/*unused*/, const Common::PatchIdMapList &maps,
                                const Metadata::IvSequenceParams &/*unused*/,
                                const Metadata::IvAccessUnitParams &/*unused*/,
                                const Metadata::ViewParams &/*unused*/) -> Common::PatchIdMapList { return maps; }
};
} // namespace TMIV::Renderer

#endif
