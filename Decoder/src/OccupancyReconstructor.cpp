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

#include <TMIV/Decoder/OccupancyReconstructor.h>

using namespace std;
using namespace TMIV::Common;
using namespace TMIV::MivBitstream;

namespace TMIV::Decoder {
OccupancyReconstructor::OccupancyReconstructor(const Json & /*rootNode*/,
                                               const Json & /*componentNode*/) {}

void OccupancyReconstructor::reconstruct(AccessUnit &frame) const {
  for (auto i = 0; i <= frame.vps.vps_atlas_count_minus1(); i++) {
    auto &atlas = frame.atlas[i];
    atlas.occFrame = Occupancy10Frame{atlas.frameSize().x(), atlas.frameSize().y()};
    for (auto y = 0; y < atlas.frameSize().y(); y++) {
      for (auto x = 0; x < atlas.frameSize().x(); x++) {
        auto patchId = atlas.patchId(y, x);
        if (patchId == unusedPatchId) {
          atlas.occFrame.getPlane(0)(y, x) = 0;
        } else if (!frame.vps.vps_occupancy_video_present_flag(i)) {
          if (frame.vps.vps_miv_extension().vme_embedded_occupancy_flag()) {
            // occupancy is embedded in geometry
            uint32_t depthOccupancyThreshold = 0;
            if (!atlas.asps.asps_miv_extension_flag() ||
                !atlas.asps.asps_miv_extension().asme_depth_occ_threshold_flag()) {
              uint16_t v = atlas.patchParamsList[patchId].pduViewIdx();
              depthOccupancyThreshold =
                  frame.viewParamsList[v].dq.dq_depth_occ_map_threshold_default();
            } else {
              depthOccupancyThreshold = *atlas.patchParamsList[patchId].pduDepthOccMapThreshold();
            }
            atlas.occFrame.getPlane(0)(y, x) =
                (atlas.geoFrame.getPlane(0)(y, x) < depthOccupancyThreshold) ? 0 : 1;
          } else {
            // no occupancy information is available (i.e. atlas is complete)
            atlas.occFrame.getPlane(0)(y, x) = 1;
          }
        } else {
          // occupancy is signaled explicitly
          int asmeOccupancyFrameScaleFactorX =
              atlas.asps.asps_miv_extension().asme_occupancy_scale_factor_x_minus1() + 1;
          int asmeOccupancyFrameScaleFactorY =
              atlas.asps.asps_miv_extension().asme_occupancy_scale_factor_y_minus1() + 1;
          atlas.occFrame.getPlane(0)(y, x) = atlas.decOccFrame.getPlane(0)(
              y / asmeOccupancyFrameScaleFactorY, x / asmeOccupancyFrameScaleFactorX);
        }
      }
    }
  }
}
} // namespace TMIV::Decoder