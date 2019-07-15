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

#ifndef _TMIV_VIEWOPTIMIZER_VIEWREDUCER_H_
#define _TMIV_VIEWOPTIMIZER_VIEWREDUCER_H_

#include <TMIV/Common/Json.h>
#include <TMIV/ViewOptimizer/IViewOptimizer.h>
#include <vector>

namespace TMIV::ViewOptimizer {
// The ViewOptimizer of TMIV 1.0 provided by Zhejiang University
class ViewReducer : public IViewOptimizer {
private:
  std::vector<bool> m_priorities;

public:
  ViewReducer(const Common::Json & /*unused*/, const Common::Json & /*unused*/);
  ViewReducer(const ViewReducer &) = default;
  ViewReducer(ViewReducer &&) = default;
  ViewReducer &operator=(const ViewReducer &) = default;
  ViewReducer &operator=(ViewReducer &&) = default;
  ~ViewReducer() override = default;

  auto optimizeIntraPeriod(Metadata::CameraParametersList cameras)
      -> Output<Metadata::CameraParametersList> override;

  auto optimizeFrame(Common::MVD16Frame views) const
      -> Output<Common::MVD16Frame> override;

private:
  auto calculateFOV(Metadata::CameraParameters camera) -> float;

  auto calculateDistance(Metadata::CameraParameters camera_1,
                         Metadata::CameraParameters camera_2) -> float;

  auto calculateOverlapping(Metadata::CameraParameters camera_from,
                            Metadata::CameraParameters camera_to) -> float;
};
} // namespace TMIV::ViewOptimizer

#endif
