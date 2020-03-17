/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

#ifndef _TMIV_VIEWOPTIMIZER_VIEWREDUCER_H_
#define _TMIV_VIEWOPTIMIZER_VIEWREDUCER_H_

#include <TMIV/ViewOptimizer/IViewOptimizer.h>

#include <TMIV/Common/Json.h>
#include <vector>

namespace TMIV::ViewOptimizer {
class ViewReducer : public IViewOptimizer {
private:
  std::vector<bool> m_isBasicView;

public:
  ViewReducer(const Common::Json & /*unused*/, const Common::Json & /*unused*/);
  ViewReducer(const ViewReducer &) = default;
  ViewReducer(ViewReducer &&) = default;
  ViewReducer &operator=(const ViewReducer &) = default;
  ViewReducer &operator=(ViewReducer &&) = default;
  ~ViewReducer() override = default;

  auto optimizeSequence(MivBitstream::IvSequenceParams ivSequenceParams) -> Output override;

  auto optimizeFrame(Common::MVD16Frame views) const -> Common::MVD16Frame override {
    return views;
  }

private:
  static auto calculateFOV(const MivBitstream::ViewParams &viewParams) -> float;

  static auto calculateDistance(const MivBitstream::ViewParams &camera_1,
                                const MivBitstream::ViewParams &camera_2) -> float;

  static auto calculateOverlapping(const MivBitstream::ViewParams &camera_from,
                                   const MivBitstream::ViewParams &camera_to) -> float;
};
} // namespace TMIV::ViewOptimizer

#endif
