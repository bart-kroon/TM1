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

#include <TMIV/Renderer/NoSynthesizer.h>

#include <algorithm>
#include <iostream>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Renderer {
NoSynthesizer::NoSynthesizer(const Json & /*unused*/, const Json & /*componentNode*/) {}

auto NoSynthesizer::renderFrame(const Decoder::AccessUnit &frame,
                                const ViewParams &viewportParams) const -> Texture444Depth16Frame {
  auto viewport = Texture444Depth16Frame{
      Texture444Frame{viewportParams.ci.ci_projection_plane_width_minus1() + 1,
                      viewportParams.ci.ci_projection_plane_height_minus1() + 1},
      Depth16Frame{viewportParams.ci.ci_projection_plane_width_minus1() + 1,
                   viewportParams.ci.ci_projection_plane_height_minus1() + 1}};

  assert(!frame.atlas.empty());
  const auto &atlas = frame.atlas.front();

  assert(atlas.attrFrame.getSize() == atlas.frameSize());
  assert(atlas.geoFrame.getSize() == atlas.frameSize());

  const auto rows = min<int>(viewportParams.ci.ci_projection_plane_height_minus1() + 1,
                             atlas.asps.asps_frame_height());
  const auto cols = min<int>(viewportParams.ci.ci_projection_plane_width_minus1() + 1,
                             atlas.asps.asps_frame_width());

  cout << "NoSynthesizer: " << rows << " x " << cols << '\n';

  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      for (int d = 0; d < 3; ++d) {
        viewport.first.getPlane(d)(y, x) = atlas.attrFrame.getPlane(d)(y, x);
      }
      viewport.second.getPlane(0)(y, x) =
          static_cast<uint16_t>(atlas.geoFrame.getPlane(0)(y, x) * 64);
    }
  }

  return viewport;
}
} // namespace TMIV::Renderer
