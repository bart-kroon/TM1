/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ISO/IEC
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

#include <TMIV/Pruner/NoPruner.h>

namespace TMIV::Pruner {
NoPruner::NoPruner(const Common::Json & /* rootConfig */, const Common::Json & /* nodeConfig */) {}

auto NoPruner::prepareSequence(const PrunerParams & /* params */)
    -> std::vector<MivBitstream::PruningParents> {
  return {};
}

auto NoPruner::getPixelInformation() -> Common::FrameList<uint32_t> { return {}; }

auto NoPruner::prune(MivBitstream::ViewParamsList &viewParamsList,
                     const Common::DeepFrameList & /* views */, int32_t /*semiBasicCount*/)
    -> Common::FrameList<uint8_t> {
  auto mask = Common::FrameList<uint8_t>(viewParamsList.size());
  transform(viewParamsList.cbegin(), viewParamsList.cend(), mask.begin(),
            [](const MivBitstream::ViewParams &vp) {
              auto mask_ =
                  Common::Frame<uint8_t>::lumaOnly({vp.ci.ci_projection_plane_width_minus1() + 1,
                                                    vp.ci.ci_projection_plane_height_minus1() + 1});
              mask_.fillOne();
              return mask_;
            });
  return mask;
}
} // namespace TMIV::Pruner
